// SketchSoC integration top (paper Sec. 3, Fig. 5): the full query-
// lifecycle accelerator behind one AXI4-Lite control port.
//
// Contents (all wired exactly as the tb_ctrl_plane harness that
// validates them):
//   work_mux         packet ingress vs. migration replay arbitration
//   smu_exec         sketch execution engine (CMS/HH/Bloom/HLL apply)
//   mig_engine       tier migration engine (freeze/copy/commit + replay)
//   tier_mgr         heat scanner + promote/demote decider + free pools
//   state_directory  per-query block directory (4 read ports, 2 write)
//   heat_table       per-block heat counters
//   l0_store         on-chip SRAM tier (L0)
//   l1_arb2          L1 port arbiter (exec m0 / engine m1 -> backing AXI)
//   ctrl_plane       AXI4-Lite registers, install/remove/readout FSMs
//
// External interfaces:
//   s_axi_*  AXI4-Lite slave: register map documented in ctrl_plane.sv
//   ing_*    packet ingress stream, one 256-bit pkt_desc_t flit/packet,
//            backpressured by ing_rdy (drops low during remove/migration
//            quiesce windows -- sources must hold off, nothing is lost)
//   wo_*     work-item completion strobe (buffer-free / debug hook)
//   m_axi_*  L1 backing store master, 32-bit byte address, 64-bit data
//            (no RESP channels: the arbiter target is always OKAY --
//            on Alveo this maps onto an HBM AXI port via a data-width
//            converter; see docs/board_notes)
//
// Timing closure target: xcu280 @ 220 MHz.

`timescale 1ns/1ps

module sketchsoc_top
  import sketchsoc_pkg::*;
#(
  parameter int unsigned QBLK_AW = 4,      // blocks/query = 2^QBLK_AW
  parameter int unsigned L0_AW   = 11,     // L0 words    = 2^L0_AW
  parameter int unsigned PROM_TH = 512,    // promote heat threshold
  parameter int unsigned DEM_TH  = 32,     // demote  heat threshold
  parameter logic [3:0]  ING_SLOT = 4'd15  // ingress work-item tag
) (
  input  logic        clk,
  input  logic        rst_n,

  // ---- AXI4-Lite slave (control / status / telemetry) -----------------
  input  logic [11:0] s_axi_awaddr,
  input  logic [2:0]  s_axi_awprot,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [11:0] s_axi_araddr,
  input  logic [2:0]  s_axi_arprot,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  // ---- packet ingress ---------------------------------------------------
  input  logic        ing_val,
  output logic        ing_rdy,
  input  pkt_desc_t   ing_pkt,

  // ---- work-item completion --------------------------------------------
  output logic        wo_val,
  output logic [3:0]  wo_slot,
  output logic        wo_is_token,

  // ---- L1 backing store (AXI master, 64-bit data) -----------------------
  output logic        m_axi_arvalid,
  input  logic        m_axi_arready,
  output logic [31:0] m_axi_araddr,
  input  logic        m_axi_rvalid,
  output logic        m_axi_rready,
  input  logic [63:0] m_axi_rdata,
  output logic        m_axi_awvalid,
  input  logic        m_axi_awready,
  output logic [31:0] m_axi_awaddr,
  output logic [63:0] m_axi_awdata,
  output logic [7:0]  m_axi_awstrb,
  input  logic        m_axi_bvalid,
  output logic        m_axi_bready
);

  localparam int unsigned DIR_IDXW = QID_W + QBLK_AW;

  // ------------------------------------------------------------------
  // inter-module signals (naming follows the block diagram: e_=exec,
  // g_=engine (migration), t_=tier manager, c_=control plane, b_=bypass)
  // ------------------------------------------------------------------
  qdesc_t      qd [QNUM];
  logic [7:0]  qgen [QNUM];

  // mux -> exec
  logic        wi_val, wi_rdy, wi_is_token, wi_gen_ok;
  pkt_desc_t   wi_pkt;
  token_t      wi_token;
  logic [2:0]  wi_ovr_qid;
  logic [15:0] wi_ovr_blk;
  logic [31:0] wi_ovr_addr;
  logic [1:0]  wi_ovr_tier;
  logic [3:0]  wi_slot;

  // exec -> world
  logic        exec_idle;

  // exec directory (r0)
  logic                e_dr_val, e_dr_rdy, e_dr_dv;
  logic [DIR_IDXW-1:0] e_dr_idx;
  dirent_t             e_dr_ent;

  // exec L0 (r0/w0)
  logic             e_l0r_val, e_l0r_rdy, e_l0r_dv;
  logic [L0_AW-1:0] e_l0r_addr;
  logic [31:0]      e_l0r_data;
  logic             e_l0w_val, e_l0w_rdy;
  logic [L0_AW-1:0] e_l0w_addr;
  logic [31:0]      e_l0w_data;
  logic [3:0]       e_l0w_strb;

  // exec heat access
  logic                ht_acc_val, ht_acc_rdy;
  logic [DIR_IDXW-1:0] ht_acc_idx;

  // divert exec <-> engine
  logic [MIG_SLOTS-1:0] div_full;
  logic                 div_val, div_rdy;
  logic [3:0]           div_slot;
  token_t               div_tok;

  // exec L1 (arbiter m0)
  logic        e_arvalid, e_arready, e_rvalid, e_rready;
  logic [31:0] e_araddr;
  logic [63:0] e_rdata;
  logic        e_awvalid, e_awready, e_bvalid, e_bready;
  logic [31:0] e_awaddr;
  logic [63:0] e_awdata;
  logic [7:0]  e_awstrb;

  // tier-manager -> engine command (raw + cmd_hold gate)
  logic     mc_val_raw, mc_rdy, mc_val_gated;
  mig_cmd_t mc_cmd;
  logic     mig_busy;
  logic     eng_req_quiesce, req_q_mux;
  logic [31:0] mig_cnt, rep_ok_cnt, rep_drop_cnt, cmd_rej_cnt, err_cnt;
  logic        rel_val;
  logic [1:0]  rel_tier;
  logic [31:0] rel_addr;

  // engine directory (r1 read, w0 write)
  logic                g_dr_val, g_dr_rdy, g_dr_dv;
  logic [DIR_IDXW-1:0] g_dr_idx;
  dirent_t             g_dr_ent;
  logic                g_dw_val, g_dw_rdy;
  logic [DIR_IDXW-1:0] g_dw_idx;
  dirent_t             g_dw_ent;

  // engine L0 (r1 through the ctrl bypass, w1 direct)
  logic             g_l0r_val, g_l0r_rdy, g_l0r_dv;
  logic [L0_AW-1:0] g_l0r_addr;
  logic [31:0]      g_l0r_data;
  logic             g_l0w_val, g_l0w_rdy;
  logic [L0_AW-1:0] g_l0w_addr;
  logic [31:0]      g_l0w_data;
  logic [3:0]       g_l0w_strb;

  // engine L1 (arbiter m1 through the ctrl bypass)
  logic        g_arvalid, g_arready, g_rvalid, g_rready;
  logic [31:0] g_araddr;
  logic [63:0] g_rdata;
  logic        g_awvalid, g_awready, g_bvalid, g_bready;
  logic [31:0] g_awaddr;
  logic [63:0] g_awdata;
  logic [7:0]  g_awstrb;

  // engine replay -> mux
  logic        rp_val, rp_rdy, rp_gen_ok;
  token_t      rp_tok;
  logic [2:0]  rp_ovr_qid;
  logic [15:0] rp_ovr_blk;
  logic [31:0] rp_ovr_addr;
  logic [1:0]  rp_ovr_tier;
  logic [3:0]  rp_slot;

  // tier manager <-> directory r2 and heat scan
  logic                t_dr_val, t_dr_rdy, t_dr_dv;
  logic [DIR_IDXW-1:0] t_dr_idx;
  dirent_t             t_dr_ent;
  logic                h_scn_val, h_scn_rdy, h_scn_dv;
  logic [DIR_IDXW-1:0] h_scn_idx;
  logic                h_scn_decay;
  logic [15:0]         h_scn_heat;

  // ctrl <-> tier manager pool interface
  logic        cp_push_val;
  logic [1:0]  cp_push_tier;
  logic [31:0] cp_push_addr;
  logic        cp_flush;
  logic        cp_scn;
  logic        cp_pop_val;
  logic [1:0]  cp_pop_tier;
  logic        tm_pop_rdy;
  logic [31:0] tm_pop_addr;
  logic        tm_pop_empty;

  // ctrl directory (r3 read, w1 write)
  logic                c_dr_val, c_dr_rdy, c_dr_dv;
  logic [DIR_IDXW-1:0] c_dr_idx;
  dirent_t             c_dr_ent;
  logic                c_dw_val, c_dw_rdy;
  logic [DIR_IDXW-1:0] c_dw_idx;
  dirent_t             c_dw_ent;

  // ctrl bypass store-side ports
  logic             b_l0r_val, b_l0r_rdy, b_l0r_dv;
  logic [L0_AW-1:0] b_l0r_addr;
  logic [31:0]      b_l0r_data;
  logic             b_l1_arvalid, b_l1_arready, b_l1_rvalid, b_l1_rready;
  logic [31:0]      b_l1_araddr;
  logic [63:0]      b_l1_rdata;
  logic             b_l1_awvalid, b_l1_awready, b_l1_bvalid, b_l1_bready;
  logic [31:0]      b_l1_awaddr;
  logic [63:0]      b_l1_awdata;
  logic [7:0]       b_l1_awstrb;

  // exec accounting (acc_sel driven by ctrl_plane)
  logic [4:0]  acc_sel;
  logic [31:0] acc_rd;

  // tier-manager telemetry
  logic [31:0] l0_free_cnt, l1_free_cnt, scan_passes, prom_cnt, dem_cnt;

  logic        cp_cmd_hold, cp_quiesce;

  // cmd_hold: the readout FSM parks new migration commands while a sketch
  // readout owns the store ports; quiesce: remove/migration drain ingress
  assign mc_val_gated = mc_val_raw && !cp_cmd_hold;
  assign req_q_mux    = eng_req_quiesce | cp_quiesce;

  // ------------------------------------------------------------------
  // work mux
  // ------------------------------------------------------------------
  work_mux u_mux (
    .clk(clk), .rst_n(rst_n),
    .ing_val(ing_val), .ing_rdy(ing_rdy), .ing_pkt(ing_pkt),
    .ing_slot(ING_SLOT),
    .rp_val(rp_val), .rp_rdy(rp_rdy), .rp_tok(rp_tok),
    .rp_gen_ok(rp_gen_ok),
    .rp_ovr_qid(rp_ovr_qid), .rp_ovr_blk(rp_ovr_blk),
    .rp_ovr_addr(rp_ovr_addr), .rp_ovr_tier(rp_ovr_tier),
    .rp_slot(rp_slot),
    .req_quiesce(req_q_mux),
    .wi_val(wi_val), .wi_rdy(wi_rdy), .wi_pkt(wi_pkt),
    .wi_is_token(wi_is_token), .wi_token(wi_token), .wi_gen_ok(wi_gen_ok),
    .wi_ovr_qid(wi_ovr_qid), .wi_ovr_blk(wi_ovr_blk),
    .wi_ovr_addr(wi_ovr_addr), .wi_ovr_tier(wi_ovr_tier),
    .wi_slot(wi_slot)
  );

  // ------------------------------------------------------------------
  // execution engine
  // ------------------------------------------------------------------
  smu_exec #(.QBLK_AW(QBLK_AW), .L0_AW(L0_AW)) u_exec (
    .clk(clk), .rst_n(rst_n),
    .wi_val(wi_val), .wi_rdy(wi_rdy), .wi_pkt(wi_pkt),
    .wi_is_token(wi_is_token), .wi_token(wi_token), .wi_gen_ok(wi_gen_ok),
    .wi_ovr_qid(wi_ovr_qid), .wi_ovr_blk(wi_ovr_blk),
    .wi_ovr_addr(wi_ovr_addr), .wi_ovr_tier(wi_ovr_tier),
    .wi_slot(wi_slot),
    .wo_val(wo_val), .wo_slot(wo_slot), .wo_is_token(wo_is_token),
    .idle(exec_idle),
    .qd(qd),
    .dr_val(e_dr_val), .dr_rdy(e_dr_rdy), .dr_idx(e_dr_idx),
    .dr_ent(e_dr_ent), .dr_dv(e_dr_dv),
    .l0r_val(e_l0r_val), .l0r_rdy(e_l0r_rdy), .l0r_addr(e_l0r_addr),
    .l0r_data(e_l0r_data), .l0r_dv(e_l0r_dv),
    .l0w_val(e_l0w_val), .l0w_rdy(e_l0w_rdy), .l0w_addr(e_l0w_addr),
    .l0w_data(e_l0w_data), .l0w_strb(e_l0w_strb),
    .ht_val(ht_acc_val), .ht_rdy(ht_acc_rdy), .ht_idx(ht_acc_idx),
    .div_full(div_full), .div_val(div_val), .div_slot(div_slot),
    .div_tok(div_tok), .div_rdy(div_rdy),
    .l1_arvalid(e_arvalid), .l1_arready(e_arready), .l1_araddr(e_araddr),
    .l1_rvalid(e_rvalid), .l1_rready(e_rready), .l1_rdata(e_rdata),
    .l1_awvalid(e_awvalid), .l1_awready(e_awready), .l1_awaddr(e_awaddr),
    .l1_awdata(e_awdata), .l1_awstrb(e_awstrb),
    .l1_bvalid(e_bvalid), .l1_bready(e_bready),
    .acc_sel(acc_sel), .acc_rd(acc_rd),
    .cand_q(3'd0), .cand_i(4'd0), .cand_rd()
  );

  // ------------------------------------------------------------------
  // migration engine
  // ------------------------------------------------------------------
  mig_engine #(.QBLK_AW(QBLK_AW), .L0_AW(L0_AW)) u_mig (
    .clk(clk), .rst_n(rst_n),
    .cmd_val(mc_val_gated), .cmd_rdy(mc_rdy), .cmd(mc_cmd),
    .qgen(qgen),
    .exec_idle(exec_idle), .req_quiesce(eng_req_quiesce),
    .dr_val(g_dr_val), .dr_rdy(g_dr_rdy), .dr_idx(g_dr_idx),
    .dr_ent(g_dr_ent), .dr_dv(g_dr_dv),
    .dw_val(g_dw_val), .dw_rdy(g_dw_rdy), .dw_idx(g_dw_idx),
    .dw_ent(g_dw_ent),
    .l0r_val(g_l0r_val), .l0r_rdy(g_l0r_rdy), .l0r_addr(g_l0r_addr),
    .l0r_data(g_l0r_data), .l0r_dv(g_l0r_dv),
    .l0w_val(g_l0w_val), .l0w_rdy(g_l0w_rdy), .l0w_addr(g_l0w_addr),
    .l0w_data(g_l0w_data), .l0w_strb(g_l0w_strb),
    .l1_arvalid(g_arvalid), .l1_arready(g_arready),
    .l1_araddr(g_araddr),
    .l1_rvalid(g_rvalid), .l1_rready(g_rready), .l1_rdata(g_rdata),
    .l1_awvalid(g_awvalid), .l1_awready(g_awready),
    .l1_awaddr(g_awaddr), .l1_awdata(g_awdata), .l1_awstrb(g_awstrb),
    .l1_bvalid(g_bvalid), .l1_bready(g_bready),
    .div_full(div_full), .div_val(div_val), .div_slot(div_slot),
    .div_tok(div_tok), .div_rdy(div_rdy),
    .rp_val(rp_val), .rp_rdy(rp_rdy), .rp_tok(rp_tok),
    .rp_gen_ok(rp_gen_ok),
    .rp_ovr_qid(rp_ovr_qid), .rp_ovr_blk(rp_ovr_blk),
    .rp_ovr_addr(rp_ovr_addr), .rp_ovr_tier(rp_ovr_tier),
    .rp_slot(rp_slot),
    .wo_val(wo_val), .wo_slot(wo_slot), .wo_is_token(wo_is_token),
    .rel_val(rel_val), .rel_tier(rel_tier), .rel_addr(rel_addr),
    .busy(mig_busy),
    .mig_cnt(mig_cnt), .rep_ok_cnt(rep_ok_cnt),
    .rep_drop_cnt(rep_drop_cnt), .cmd_rej_cnt(cmd_rej_cnt),
    .err_cnt(err_cnt)
  );

  // ------------------------------------------------------------------
  // tier manager
  // ------------------------------------------------------------------
  tier_mgr #(.QBLK_AW(QBLK_AW), .PROM_TH(PROM_TH), .DEM_TH(DEM_TH)) u_tm (
    .clk(clk), .rst_n(rst_n),
    .scn_en(cp_scn),
    .dr_val(t_dr_val), .dr_rdy(t_dr_rdy), .dr_idx(t_dr_idx),
    .dr_ent(t_dr_ent), .dr_dv(t_dr_dv),
    .ht_val(h_scn_val), .ht_rdy(h_scn_rdy), .ht_idx(h_scn_idx),
    .ht_decay(h_scn_decay), .ht_heat(h_scn_heat), .ht_dv(h_scn_dv),
    .cmd_val(mc_val_raw), .cmd_rdy(mc_rdy), .cmd(mc_cmd),
    .mig_busy(mig_busy),
    .rel_val(rel_val), .rel_tier(rel_tier), .rel_addr(rel_addr),
    .push_val(cp_push_val), .push_tier(cp_push_tier),
    .push_addr(cp_push_addr),
    .flush_stacks(cp_flush),
    .pop_val(cp_pop_val), .pop_tier(cp_pop_tier),
    .pop_rdy(tm_pop_rdy), .pop_addr(tm_pop_addr), .pop_empty(tm_pop_empty),
    .l0_free_cnt(l0_free_cnt), .l1_free_cnt(l1_free_cnt),
    .scan_passes(scan_passes), .prom_cnt(prom_cnt), .dem_cnt(dem_cnt)
  );

  // ------------------------------------------------------------------
  // L1 port arbiter: exec m0 / engine m1 (via ctrl bypass) -> backing AXI
  // ------------------------------------------------------------------
  l1_arb2 u_arb (
    .clk(clk), .rst_n(rst_n),
    .m0_arvalid(e_arvalid), .m0_arready(e_arready), .m0_araddr(e_araddr),
    .m0_rvalid(e_rvalid), .m0_rready(e_rready), .m0_rdata(e_rdata),
    .m0_awvalid(e_awvalid), .m0_awready(e_awready), .m0_awaddr(e_awaddr),
    .m0_awdata(e_awdata), .m0_awstrb(e_awstrb),
    .m0_bvalid(e_bvalid), .m0_bready(e_bready),
    .m1_arvalid(b_l1_arvalid), .m1_arready(b_l1_arready),
    .m1_araddr(b_l1_araddr),
    .m1_rvalid(b_l1_rvalid), .m1_rready(b_l1_rready),
    .m1_rdata(b_l1_rdata),
    .m1_awvalid(b_l1_awvalid), .m1_awready(b_l1_awready),
    .m1_awaddr(b_l1_awaddr),
    .m1_awdata(b_l1_awdata), .m1_awstrb(b_l1_awstrb),
    .m1_bvalid(b_l1_bvalid), .m1_bready(b_l1_bready),
    .s_arvalid(m_axi_arvalid), .s_arready(m_axi_arready),
    .s_araddr(m_axi_araddr),
    .s_rvalid(m_axi_rvalid), .s_rready(m_axi_rready),
    .s_rdata(m_axi_rdata),
    .s_awvalid(m_axi_awvalid), .s_awready(m_axi_awready),
    .s_awaddr(m_axi_awaddr),
    .s_awdata(m_axi_awdata), .s_awstrb(m_axi_awstrb),
    .s_bvalid(m_axi_bvalid), .s_bready(m_axi_bready)
  );

  // ------------------------------------------------------------------
  // L0 on-chip store
  // ------------------------------------------------------------------
  l0_store #(.AW(L0_AW)) u_l0 (
    .clk(clk), .rst_n(rst_n),
    .r0_val(e_l0r_val), .r0_rdy(e_l0r_rdy), .r0_addr(e_l0r_addr),
    .r0_data(e_l0r_data), .r0_dv(e_l0r_dv),
    .r1_val(b_l0r_val), .r1_rdy(b_l0r_rdy), .r1_addr(b_l0r_addr),
    .r1_data(b_l0r_data), .r1_dv(b_l0r_dv),
    .w0_val(e_l0w_val), .w0_rdy(e_l0w_rdy), .w0_addr(e_l0w_addr),
    .w0_data(e_l0w_data), .w0_strb(e_l0w_strb),
    .w1_val(g_l0w_val), .w1_rdy(g_l0w_rdy), .w1_addr(g_l0w_addr),
    .w1_data(g_l0w_data), .w1_strb(g_l0w_strb)
  );

  // ------------------------------------------------------------------
  // state directory
  // ------------------------------------------------------------------
  state_directory #(.QBLK_AW(QBLK_AW)) u_dir (
    .clk(clk), .rst_n(rst_n),
    .r0_val(e_dr_val), .r0_rdy(e_dr_rdy), .r0_idx(e_dr_idx),
    .r0_ent(e_dr_ent), .r0_dv(e_dr_dv),
    .r1_val(g_dr_val), .r1_rdy(g_dr_rdy), .r1_idx(g_dr_idx),
    .r1_ent(g_dr_ent), .r1_dv(g_dr_dv),
    .r2_val(t_dr_val), .r2_rdy(t_dr_rdy), .r2_idx(t_dr_idx),
    .r2_ent(t_dr_ent), .r2_dv(t_dr_dv),
    .r3_val(c_dr_val), .r3_rdy(c_dr_rdy), .r3_idx(c_dr_idx),
    .r3_ent(c_dr_ent), .r3_dv(c_dr_dv),
    .w0_val(g_dw_val), .w0_rdy(g_dw_rdy), .w0_idx(g_dw_idx),
    .w0_ent(g_dw_ent),
    .w1_val(c_dw_val), .w1_rdy(c_dw_rdy), .w1_idx(c_dw_idx),
    .w1_ent(c_dw_ent)
  );

  // ------------------------------------------------------------------
  // heat table
  // ------------------------------------------------------------------
  heat_table #(.QBLK_AW(QBLK_AW)) u_heat (
    .clk(clk), .rst_n(rst_n),
    .acc_val(ht_acc_val), .acc_rdy(ht_acc_rdy), .acc_idx(ht_acc_idx),
    .scn_val(h_scn_val), .scn_rdy(h_scn_rdy), .scn_idx(h_scn_idx),
    .scn_decay(h_scn_decay), .scn_heat(h_scn_heat), .scn_dv(h_scn_dv)
  );

  // ------------------------------------------------------------------
  // control plane
  // ------------------------------------------------------------------
  ctrl_plane #(.QBLK_AW(QBLK_AW), .L0_AW(L0_AW)) u_ctrl (
    .clk(clk), .rst_n(rst_n),
    .s_axi_awaddr(s_axi_awaddr), .s_axi_awprot(s_axi_awprot),
    .s_axi_awvalid(s_axi_awvalid), .s_axi_awready(s_axi_awready),
    .s_axi_wdata(s_axi_wdata), .s_axi_wstrb(s_axi_wstrb),
    .s_axi_wvalid(s_axi_wvalid), .s_axi_wready(s_axi_wready),
    .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(s_axi_bready),
    .s_axi_araddr(s_axi_araddr), .s_axi_arprot(s_axi_arprot),
    .s_axi_arvalid(s_axi_arvalid), .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(s_axi_rready),
    .qd_o(qd), .qgen_o(qgen),
    .dw_val(c_dw_val), .dw_rdy(c_dw_rdy), .dw_idx(c_dw_idx),
    .dw_ent(c_dw_ent),
    .dr_val(c_dr_val), .dr_rdy(c_dr_rdy), .dr_idx(c_dr_idx),
    .dr_ent(c_dr_ent), .dr_dv(c_dr_dv),
    .push_val(cp_push_val), .push_tier(cp_push_tier),
    .push_addr(cp_push_addr),
    .flush_stacks(cp_flush), .scn_en_o(cp_scn),
    .pop_val(cp_pop_val), .pop_tier(cp_pop_tier),
    .pop_rdy(tm_pop_rdy), .pop_addr(tm_pop_addr), .pop_empty(tm_pop_empty),
    .rel_val(rel_val), .tm_cmd_val(mc_val_raw),
    .cmd_hold(cp_cmd_hold), .quiesce_req_o(cp_quiesce),
    .mig_busy(mig_busy), .exec_idle(exec_idle),
    .eng_l0r_val(g_l0r_val), .eng_l0r_rdy(g_l0r_rdy),
    .eng_l0r_addr(g_l0r_addr), .eng_l0r_data(g_l0r_data),
    .eng_l0r_dv(g_l0r_dv),
    .st_l0r_val(b_l0r_val), .st_l0r_rdy(b_l0r_rdy),
    .st_l0r_addr(b_l0r_addr), .st_l0r_data(b_l0r_data),
    .st_l0r_dv(b_l0r_dv),
    .eng_l1_arvalid(g_arvalid), .eng_l1_arready(g_arready),
    .eng_l1_araddr(g_araddr),
    .eng_l1_rvalid(g_rvalid), .eng_l1_rready(g_rready),
    .eng_l1_rdata(g_rdata),
    .eng_l1_awvalid(g_awvalid), .eng_l1_awready(g_awready),
    .eng_l1_awaddr(g_awaddr), .eng_l1_awdata(g_awdata),
    .eng_l1_awstrb(g_awstrb),
    .eng_l1_bvalid(g_bvalid), .eng_l1_bready(g_bready),
    .st_l1_arvalid(b_l1_arvalid), .st_l1_arready(b_l1_arready),
    .st_l1_araddr(b_l1_araddr),
    .st_l1_rvalid(b_l1_rvalid), .st_l1_rready(b_l1_rready),
    .st_l1_rdata(b_l1_rdata),
    .st_l1_awvalid(b_l1_awvalid), .st_l1_awready(b_l1_awready),
    .st_l1_awaddr(b_l1_awaddr), .st_l1_awdata(b_l1_awdata),
    .st_l1_awstrb(b_l1_awstrb),
    .st_l1_bvalid(b_l1_bvalid), .st_l1_bready(b_l1_bready),
    .mig_cnt(mig_cnt), .rep_ok_cnt(rep_ok_cnt),
    .rep_drop_cnt(rep_drop_cnt),
    .cmd_rej_cnt(cmd_rej_cnt), .err_cnt(err_cnt),
    .prom_cnt(prom_cnt), .dem_cnt(dem_cnt), .scan_passes(scan_passes),
    .l0_free_cnt(l0_free_cnt), .l1_free_cnt(l1_free_cnt),
    .acc_sel(acc_sel), .acc_rd(acc_rd)
  );

endmodule
