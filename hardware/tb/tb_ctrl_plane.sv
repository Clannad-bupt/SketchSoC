// Control-plane regression: the query lifecycle runs in hardware.
//
// DUT set (all real, no stubs -- same fabric as tb_tier_mgr plus ctrl_plane):
//
//   AXI-Lite master (TB) ==> ctrl_plane ==> qd/qgen, directory w1/r3,
//                            tier-mgr push/pop/flush/scan-enable, bypasses
//   packets (TB) --\                          /-> tier_mgr (scan, stacks)
//                     work_mux -> smu_exec <-/       |  cmd (gated by cmd_hold)
//                     replay <--- mig_engine <-------+  L0 r1 / L1 m1 through
//                                                  |     the ctrl bypasses
//                                                  +-> release -> stacks
//
// Everything tb_tier_mgr did with TB probes (install directory entries,
// drive free-stack pushes, toggle the scan) now happens through the register
// interface, which is the deliverable this bench protects.
//
// Reference model: identical to tb_tier_mgr with one upgrade -- the engine's
// directory CAS port (w0) is monitored so the shadow directory tracks
// freeze/replaying/commit in real time, and the block copy is mirrored at
// the FIRST REPLAY HANDSHAKE (the engine replays only after the copy
// completes) or, for zero-replay migrations, at the commit CAS.  The old
// lazy-mirror-at-wo_token would lose post-commit writes when a migration
// had no diverted packets.
//
// What the cases cover:
//   CP1  reset state, CTRL/QSEL write-readback, W-before-AW ordering,
//        unmapped read, POOL seed + flush accept path.
//   CP2  pool seeding through POOL_PUSH (deferred write acknowledge), live
//        free counts, telemetry snapshot of globals, r3 read of an invalid
//        directory entry.
//   CP3  install q0 (2-block L1 CMS): staged descriptor commit, blocks
//        popped from the L1 stack (LIFO order checked against a shadow
//        stack model), directory entries readable through DIR_IDX/DIR_ENT,
//        enable published last, 50 packets applied.
//   CP4  ONLINE install of q1 while q0 traffic flows.  The pump pauses only
//        around the publish edge so no packet straddles enable 0->1 (the
//        reference model could not otherwise tell which side a mid-flight
//        packet landed on); the install FSM itself runs against live state.
//   CP5  remove q0: epoch bump + disable, quiesce, 16-block walk invalidates
//        every entry, blocks returned to the pool in LIFO order, gen+1,
//        removed query is silent (att flat -- skipped, not rejected).
//   CP6  sketch readout through the engine-port bypasses: an L1 line and an
//        L0 word read back through RO_CMD vs the physical arrays, plus an
//        L0 install/remove round trip (pool return path).
//   CP7  quota rollback: install asking for more blocks than the pool holds
//        winds back cleanly (entries invalidated, addresses pushed back in
//        original LIFO order, op_err sticky until W1C), then a normal
//        install succeeds on the same query.
//   CP8  scan re-enable: heat-driven promote with traffic and a readout
//        interleaved mid-migration (the readout FSM parks on mig_busy under
//        cmd_hold), 100-packet coexistence with the scan running, telemetry
//        snapshot equal to the live counters.

`timescale 1ns/1ps

module tb_ctrl_plane;

  import sketchsoc_pkg::*;

  localparam int unsigned QBLK_AW  = 4;
  localparam int unsigned L0_AW    = 11;
  localparam int unsigned L0_WORDS = 1 << L0_AW;
  localparam int unsigned L1_LINES = 2048;
  localparam int unsigned DIR_IDXW = QID_W + QBLK_AW;
  localparam int unsigned PROM_TH  = 512;   // below DUT defaults so a few
  localparam int unsigned DEM_TH   = 32;    // hundred packets suffice

  // register map (must mirror ctrl_plane.sv)
  localparam logic [11:0] A_CTRL   = 12'h000, A_STATUS = 12'h004,
                          A_QSEL   = 12'h008, A_QD0    = 12'h00C,
                          A_QS0    = 12'h010, A_QS1    = 12'h014,
                          A_QS2    = 12'h018, A_QS3    = 12'h01C,
                          A_QDHI   = 12'h020, A_QCFG   = 12'h024,
                          A_QCMD   = 12'h028, A_POOL   = 12'h02C,
                          A_DIRIDX = 12'h030, A_DENTLO = 12'h034,
                          A_DENTHI = 12'h038, A_ROCMD  = 12'h03C,
                          A_ROLO   = 12'h040, A_ROHI   = 12'h044,
                          A_TLM    = 12'h048;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  int errors = 0;

  // ------------------------------------------------------------------
  // signals
  // ------------------------------------------------------------------
  // query descriptors: driven by ctrl_plane, consumed by exec + engine
  qdesc_t      qd [QNUM];
  logic [7:0]  qgen [QNUM];

  // ingress (TB) -> mux
  logic      ing_val, ing_rdy;
  pkt_desc_t ing_pkt;

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
  logic       wo_val, wo_is_token, exec_idle;
  logic [3:0] wo_slot;

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

  // tier-manager -> engine command (ungated) and the cmd_hold gate
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

  // ctrl: readout gate / remove backpressure
  logic        cp_cmd_hold, cp_quiesce;

  // ctrl directory ports (w1 write, r3 read)
  logic                c_dw_val, c_dw_rdy;
  logic [DIR_IDXW-1:0] c_dw_idx;
  dirent_t             c_dw_ent;
  logic                c_dr_val, c_dr_rdy, c_dr_dv;
  logic [DIR_IDXW-1:0] c_dr_idx;
  dirent_t             c_dr_ent;

  // ctrl L0-r1 bypass outputs -> l0_store r1
  logic             b_l0r_val, b_l0r_rdy, b_l0r_dv;
  logic [L0_AW-1:0] b_l0r_addr;
  logic [31:0]      b_l0r_data;

  // ctrl L1-m1 bypass outputs -> l1_arb2 m1
  logic        b_l1_arvalid, b_l1_arready, b_l1_rvalid, b_l1_rready;
  logic [31:0] b_l1_araddr;
  logic [63:0] b_l1_rdata;
  logic        b_l1_awvalid, b_l1_awready, b_l1_bvalid, b_l1_bready;
  logic [31:0] b_l1_awaddr;
  logic [63:0] b_l1_awdata;
  logic [7:0]  b_l1_awstrb;

  // arbiter slave side -> L1 model
  logic        s_arvalid, s_arready, s_rvalid, s_rready;
  logic [31:0] s_araddr;
  logic [63:0] s_rdata;
  logic        s_awvalid, s_awready, s_bvalid, s_bready;
  logic [31:0] s_awaddr;
  logic [63:0] s_awdata;
  logic [7:0]  s_awstrb;

  // exec accounting (acc_sel driven by ctrl_plane)
  logic [4:0]  acc_sel;
  logic [31:0] acc_rd;

  // tier-manager telemetry
  logic [31:0] l0_free_cnt, l1_free_cnt, scan_passes, prom_cnt, dem_cnt;

  // AXI4-Lite master (TB): bready/rready held high, single outstanding
  logic        m_awvalid, m_wvalid, m_bready, m_arvalid, m_rready;
  logic [11:0] m_awaddr, m_araddr;
  logic [31:0] m_wdata;
  logic        s_axi_awready, s_axi_wready, s_axi_bvalid;
  logic [1:0]  s_axi_bresp;
  logic        s_axi_arready, s_axi_rvalid;
  logic [31:0] s_axi_rdata;
  logic [1:0]  s_axi_rresp;

  assign m_bready = 1'b1;
  assign m_rready = 1'b1;

  assign mc_val_gated = mc_val_raw && !cp_cmd_hold;
  assign req_q_mux = eng_req_quiesce | cp_quiesce;   // remove drains ingress

  // ------------------------------------------------------------------
  // DUTs
  // ------------------------------------------------------------------
  work_mux u_mux (
    .clk(clk), .rst_n(rst_n),
    .ing_val(ing_val), .ing_rdy(ing_rdy), .ing_pkt(ing_pkt),
    .ing_slot(4'd15),
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
    .l1_arvalid(g_arvalid), .l1_arready(g_arready), .l1_araddr(g_araddr),
    .l1_rvalid(g_rvalid), .l1_rready(g_rready), .l1_rdata(g_rdata),
    .l1_awvalid(g_awvalid), .l1_awready(g_awready), .l1_awaddr(g_awaddr),
    .l1_awdata(g_awdata), .l1_awstrb(g_awstrb),
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

  l1_arb2 u_arb (
    .clk(clk), .rst_n(rst_n),
    .m0_arvalid(e_arvalid), .m0_arready(e_arready), .m0_araddr(e_araddr),
    .m0_rvalid(e_rvalid), .m0_rready(e_rready), .m0_rdata(e_rdata),
    .m0_awvalid(e_awvalid), .m0_awready(e_awready), .m0_awaddr(e_awaddr),
    .m0_awdata(e_awdata), .m0_awstrb(e_awstrb),
    .m0_bvalid(e_bvalid), .m0_bready(e_bready),
    .m1_arvalid(b_l1_arvalid), .m1_arready(b_l1_arready), .m1_araddr(b_l1_araddr),
    .m1_rvalid(b_l1_rvalid), .m1_rready(b_l1_rready), .m1_rdata(b_l1_rdata),
    .m1_awvalid(b_l1_awvalid), .m1_awready(b_l1_awready), .m1_awaddr(b_l1_awaddr),
    .m1_awdata(b_l1_awdata), .m1_awstrb(b_l1_awstrb),
    .m1_bvalid(b_l1_bvalid), .m1_bready(b_l1_bready),
    .s_arvalid(s_arvalid), .s_arready(s_arready), .s_araddr(s_araddr),
    .s_rvalid(s_rvalid), .s_rready(s_rready), .s_rdata(s_rdata),
    .s_awvalid(s_awvalid), .s_awready(s_awready), .s_awaddr(s_awaddr),
    .s_awdata(s_awdata), .s_awstrb(s_awstrb),
    .s_bvalid(s_bvalid), .s_bready(s_bready)
  );

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

  heat_table #(.QBLK_AW(QBLK_AW)) u_heat (
    .clk(clk), .rst_n(rst_n),
    .acc_val(ht_acc_val), .acc_rdy(ht_acc_rdy), .acc_idx(ht_acc_idx),
    .scn_val(h_scn_val), .scn_rdy(h_scn_rdy), .scn_idx(h_scn_idx),
    .scn_decay(h_scn_decay), .scn_heat(h_scn_heat), .scn_dv(h_scn_dv)
  );

  ctrl_plane #(.QBLK_AW(QBLK_AW), .L0_AW(L0_AW)) u_ctrl (
    .clk(clk), .rst_n(rst_n),
    .s_axi_awaddr(m_awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(m_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(m_wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(m_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(m_bready),
    .s_axi_araddr(m_araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(m_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(m_rready),
    .qd_o(qd), .qgen_o(qgen),
    .dw_val(c_dw_val), .dw_rdy(c_dw_rdy), .dw_idx(c_dw_idx), .dw_ent(c_dw_ent),
    .dr_val(c_dr_val), .dr_rdy(c_dr_rdy), .dr_idx(c_dr_idx),
    .dr_ent(c_dr_ent), .dr_dv(c_dr_dv),
    .push_val(cp_push_val), .push_tier(cp_push_tier), .push_addr(cp_push_addr),
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
    .eng_l1_rvalid(g_rvalid), .eng_l1_rready(g_rready), .eng_l1_rdata(g_rdata),
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
    .mig_cnt(mig_cnt), .rep_ok_cnt(rep_ok_cnt), .rep_drop_cnt(rep_drop_cnt),
    .cmd_rej_cnt(cmd_rej_cnt), .err_cnt(err_cnt),
    .prom_cnt(prom_cnt), .dem_cnt(dem_cnt), .scan_passes(scan_passes),
    .l0_free_cnt(l0_free_cnt), .l1_free_cnt(l1_free_cnt),
    .acc_sel(acc_sel), .acc_rd(acc_rd)
  );

  // ------------------------------------------------------------------
  // behavioural L1 (same model as tb_smu_exec / tb_mig_engine / tb_tier_mgr)
  // ------------------------------------------------------------------
  logic [63:0] l1_mem [0:L1_LINES-1];
  initial begin
    for (int i = 0; i < L1_LINES; i++) l1_mem[i] = 64'h0;
  end

  logic [31:0] ar_addr_q, wr_addr_q;
  logic [63:0] wr_data_q;
  logic [7:0]  wr_strb_q;
  int unsigned rd_dly;
  logic        rd_pend, wr_pend;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_rvalid <= 1'b0; rd_pend <= 1'b0; rd_dly <= 0;
      s_bvalid <= 1'b0; wr_pend <= 1'b0;
    end else begin
      if (s_arvalid) begin
        rd_pend   <= 1'b1;
        rd_dly    <= 3;
        ar_addr_q <= s_araddr;
      end
      if (rd_pend && (rd_dly == 0)) begin
        if (s_rvalid && s_rready) begin
          s_rvalid <= 1'b0;
          rd_pend  <= 1'b0;
        end else begin
          s_rvalid <= 1'b1;
          s_rdata  <= l1_mem[ar_addr_q[31:3] % L1_LINES];
        end
      end else if (rd_pend) begin
        rd_dly <= rd_dly - 1;
      end
      if (s_awvalid) begin
        wr_pend   <= 1'b1;
        wr_addr_q <= s_awaddr;
        wr_data_q <= s_awdata;
        wr_strb_q <= s_awstrb;
      end
      if (wr_pend) begin
        for (int b = 0; b < 8; b++)
          if (wr_strb_q[b])
            l1_mem[wr_addr_q[31:3] % L1_LINES][8*b +: 8] <= wr_data_q[8*b +: 8];
        s_bvalid <= 1'b1;
        wr_pend  <= 1'b0;
      end
      if (s_bvalid && s_bready)
        s_bvalid <= 1'b0;
    end
  end

  assign s_arready = 1'b1;
  assign s_awready = 1'b1;

  // ------------------------------------------------------------------
  // reference model (same semantics as tb_tier_mgr)
  // ------------------------------------------------------------------
  dirent_t     sdir [int];
  logic [31:0] exp_l0 [int];
  logic [63:0] exp_l1 [int];
  int          r_att [0:QNUM-1];
  int          r_rej [0:QNUM-1];
  int          r_dir [0:QNUM-1];
  int          r_rep [0:QNUM-1];
  int          r_sat [0:QNUM-1];

  function automatic dirent_t sdir_get(input int q, input int b);
    if (sdir.exists(q*4096 + b)) return sdir[q*4096 + b];
    else                         return '0;
  endfunction

  function automatic flow_key_t gen_key(input int i);
    flow_key_t k;
    k.src_ip   = 32'h0a000000 + i;
    k.dst_ip   = 32'h0b000000 + (i * 3);
    k.src_port = 1000 + i;
    k.dst_port = 80;
    k.proto    = 17;
    k.pad      = 0;
    return k;
  endfunction

  task automatic geom(input int ci, input logic [1:0] wd,
                      output int word, output int bofs);
    case (wd)
      WID_8:  begin word = ci >> 2; bofs = ci & 3; end
      WID_16: begin word = ci >> 1; bofs = (ci & 1) << 1; end
      default: begin word = ci; bofs = 0; end
    endcase
  endtask

  function automatic logic [31:0] wmaxf(input logic [1:0] wd);
    case (wd)
      WID_8:  return 32'h000000FF;
      WID_16: return 32'h0000FFFF;
      default: return 32'hFFFFFFFF;
    endcase
  endfunction

  function automatic logic [31:0] rdc(input logic [1:0] tier,
                                      input logic [31:0] addr,
                                      input int wib, input int bofs,
                                      input logic [1:0] wd);
    logic [31:0] w32;
    if (tier == TIER_L0) begin
      w32 = exp_l0.exists(addr + wib) ? exp_l0[addr + wib] : 32'h0;
    end else begin
      logic [63:0] w64;
      logic [31:0] ba;
      ba  = addr + wib * 4;
      w64 = exp_l1.exists(ba >> 3) ? exp_l1[ba >> 3] : 64'h0;
      w32 = ba[2] ? w64[63:32] : w64[31:0];
    end
    case (wd)
      WID_8:  return {24'h0, w32 >> (bofs * 8)};
      WID_16: return {16'h0, w32 >> (bofs * 8)};
      default: return w32;
    endcase
  endfunction

  task automatic wrc(input logic [1:0] tier, input logic [31:0] addr,
                     input int wib, input int bofs, input logic [1:0] wd,
                     input logic [31:0] cval);
    logic [3:0]  strb;
    logic [31:0] sd;
    strb = cell_strb(wd, bofs[1:0]);
    sd   = cval << (bofs * 8);
    if (tier == TIER_L0) begin
      logic [31:0] w;
      w = exp_l0.exists(addr + wib) ? exp_l0[addr + wib] : 32'h0;
      for (int b = 0; b < 4; b++)
        if (strb[b]) w[8*b +: 8] = sd[8*b +: 8];
      exp_l0[addr + wib] = w;
    end else begin
      logic [63:0] w64;
      logic [31:0] ba;
      logic [7:0]  strb8;
      logic [63:0] sd64;
      ba    = addr + wib * 4;
      w64   = exp_l1.exists(ba >> 3) ? exp_l1[ba >> 3] : 64'h0;
      strb8 = {4'h0, strb} << (ba[2] ? 4 : 0);
      sd64  = {32'h0, sd} << (ba[2] ? 32 : 0);
      for (int b = 0; b < 8; b++)
        if (strb8[b]) w64[8*b +: 8] = sd64[8*b +: 8];
      exp_l1[ba >> 3] = w64;
    end
  endtask

  function automatic logic [31:0] rd_word32(input logic [1:0] tier,
                                            input logic [31:0] addr,
                                            input int wib);
    if (tier == TIER_L0) begin
      return exp_l0.exists(addr + wib) ? exp_l0[addr + wib] : 32'h0;
    end else begin
      logic [63:0] w64;
      logic [31:0] ba;
      ba  = addr + wib * 4;
      w64 = exp_l1.exists(ba >> 3) ? exp_l1[ba >> 3] : 64'h0;
      return ba[2] ? w64[63:32] : w64[31:0];
    end
  endfunction

  task automatic wr_word32(input logic [1:0] tier, input logic [31:0] addr,
                           input int wib, input logic [31:0] w);
    if (tier == TIER_L0) begin
      exp_l0[addr + wib] = w;
    end else begin
      logic [63:0] w64;
      logic [31:0] ba;
      ba  = addr + wib * 4;
      w64 = exp_l1.exists(ba >> 3) ? exp_l1[ba >> 3] : 64'h0;
      if (ba[2]) w64[63:32] = w;
      else       w64[31:0]  = w;
      exp_l1[ba >> 3] = w64;
    end
  endtask

  task automatic ref_apply(input int q, input flow_key_t key,
                           input bit is_tok, input int ovr_q,
                           input int ovr_blk, input logic [31:0] ovr_addr,
                           input logic [1:0] ovr_tier);
    qdesc_t D;
    logic [1:0]  fam, tier;
    logic [31:0] cols, h, addr, minv, v, rank, word;
    int cols_log2, rows_k;
    int cl, wl, wib, blk, bofs, bitidx, bit_;
    logic [31:0] val   [0:MAX_ROWS-1];
    logic [1:0]  t_r   [0:MAX_ROWS-1];
    logic [31:0] a_r   [0:MAX_ROWS-1];
    int w_r [0:MAX_ROWS-1];
    int b_r [0:MAX_ROWS-1];
    dirent_t e;
    begin
      D         = qd[q];
      fam       = D.kind[1:0];
      cols_log2 = D.cols_log2;
      cols      = 1 << cols_log2;
      if (!is_tok) r_att[q]++;
      case (D.kind)
        KIND_CMS, KIND_HH: begin
          for (int r = 0; r < D.rows; r++) begin
            h  = row_hash(r, fam, D.seed0, D.seed1, D.seed2, D.seed3, key);
            cl = r * cols + (h >> (32 - cols_log2));
            geom(cl, D.width, wl, bofs);
            blk = wl >> 6;
            wib = wl & 63;
            if (is_tok && (q == ovr_q) && (blk == ovr_blk)) begin
              tier = ovr_tier;
              addr = ovr_addr;
            end else begin
              e = sdir_get(q, blk);
              if (!e.valid) begin r_rej[q]++; return; end
              if (e.owner != OWN_RESIDENT) begin
                if (div_full[e.mslot]) r_rej[q]++;
                else                   r_dir[q]++;
                return;
              end
              tier = e.tier;
              addr = e.addr;
            end
            t_r[r] = tier;
            a_r[r] = addr;
            w_r[r] = wib;
            b_r[r] = bofs;
            val[r] = rdc(tier, addr, wib, bofs, D.width);
          end
          minv = val[0];
          for (int r = 1; r < D.rows; r++)
            if (val[r] < minv) minv = val[r];
          v = minv + 1;
          if (v > wmaxf(D.width)) begin v = wmaxf(D.width); r_sat[q]++; end
          for (int r = 0; r < D.rows; r++)
            wrc(t_r[r], a_r[r], w_r[r], b_r[r], D.width,
                (val[r] < v) ? v : val[r]);
          if (is_tok) r_rep[q]++;
        end
        KIND_HLL, KIND_ENT: begin
          h  = row_hash(0, fam, D.seed0, D.seed1, D.seed2, D.seed3, key);
          cl = h >> (32 - cols_log2);
          geom(cl, D.width, wl, bofs);
          blk = wl >> 6;
          wib = wl & 63;
          if (is_tok && (q == ovr_q) && (blk == ovr_blk)) begin
            tier = ovr_tier; addr = ovr_addr;
          end else begin
            e = sdir_get(q, blk);
            if (!e.valid) begin r_rej[q]++; return; end
            if (e.owner != OWN_RESIDENT) begin
              if (div_full[e.mslot]) r_rej[q]++;
              else                   r_dir[q]++;
              return;
            end
            tier = e.tier; addr = e.addr;
          end
          val[0] = rdc(tier, addr, wib, bofs, D.width);
          if (D.kind == KIND_HLL) begin
            rank = clz32(h << cols_log2);
            v = (val[0] < rank) ? rank : val[0];
          end else begin
            v = val[0] + 1;
            if (v > wmaxf(D.width)) begin v = wmaxf(D.width); r_sat[q]++; end
          end
          wrc(tier, addr, wib, bofs, D.width, v);
          if (is_tok) r_rep[q]++;
        end
        default: begin  // KIND_BLM
          rows_k = D.blm_k;
          for (int i = 0; i < rows_k; i++) begin
            h = row_hash(i, fam, D.seed0, D.seed1, D.seed2, D.seed3, key);
            bitidx = h & ((1 << (cols_log2 + 5)) - 1);
            wl   = bitidx >> 5;
            bit_ = bitidx & 31;
            blk  = wl >> 6;
            wib  = wl & 63;
            if (is_tok && (q == ovr_q) && (blk == ovr_blk)) begin
              tier = ovr_tier; addr = ovr_addr;
            end else begin
              e = sdir_get(q, blk);
              if (!e.valid) begin r_rej[q]++; return; end
              if (e.owner != OWN_RESIDENT) begin
                if (div_full[e.mslot]) r_rej[q]++;
                else                   r_dir[q]++;
                return;
              end
              tier = e.tier; addr = e.addr;
            end
            word = rd_word32(tier, addr, wib);
            word = word | (32'd1 << bit_);
            wr_word32(tier, addr, wib, word);
          end
          if (is_tok) r_rep[q]++;
        end
      endcase
    end
  endtask

  // ------------------------------------------------------------------
  // AXI4-Lite master tasks: negedge-aligned, single outstanding, bready and
  // rready held high.  Writes may present W before AW (ctrl latches both
  // independently).
  // ------------------------------------------------------------------
  task automatic axi_wr(input logic [11:0] addr, input logic [31:0] data,
                        input bit wfirst = 1'b0);
    int to;
    begin
      // drain any stale b pulse from the previous transaction
      while (s_axi_bvalid) @(negedge clk);
      m_awaddr = addr; m_wdata = data;
      if (wfirst) begin
        // W first, then AW: drive W one posedge, drop, then drive AW one posedge
        m_wvalid = 1'b1; m_awvalid = 1'b0;
        @(negedge clk);                       // >=1 posedge of wvalid&&wready -> accepted
        @(negedge clk);
        m_wvalid = 1'b0;
        m_awvalid = 1'b1;
        @(negedge clk);                       // >=1 posedge of awvalid&&awready -> accepted
        @(negedge clk);
        m_awvalid = 1'b0;
      end else begin
        m_awvalid = 1'b1; m_wvalid = 1'b1;
        @(negedge clk);                       // >=1 posedge with both valids -> both accepted
        @(negedge clk);
        m_awvalid = 1'b0; m_wvalid = 1'b0;
      end
      // the b pulse is one cycle wide; catch it (drained at entry, so this is ours)
      to = 0;
      while (!s_axi_bvalid) begin
        @(negedge clk);
        to++;
        if (to > 100000) begin
          errors++;
          $display("FAIL axi_wr: bvalid timeout wr %h bvld=%b", addr, s_axi_bvalid);
          $finish;
        end
      end
      @(negedge clk);                         // let the b pulse clear before next txn
    end
  endtask

  // AXI4-Lite read: assert araddr/arvalid, hold until arready is seen high at
  // a negedge (accept happened at the preceding posedge), then wait for the
  // one-cycle rvalid pulse and capture rdata.  bready/rready are tied high.
  task automatic axi_rd(input logic [11:0] addr, output logic [31:0] data);
    int to;
    begin
      // drain any stale r pulse from the previous transaction
      while (s_axi_rvalid) @(negedge clk);
      m_araddr  = addr;
      m_arvalid = 1'b1;
      // wait for a negedge where arready is high => accept latched; then drop
      to = 0;
      while (!s_axi_arready) begin
        @(negedge clk);
        to++;
        if (to > 100000) begin
          errors++; $display("FAIL axi_rd: arready timeout rd %h", addr); $finish;
        end
      end
      @(negedge clk);                         // span the accept posedge
      m_arvalid = 1'b0;
      // catch the one-cycle rvalid pulse (drained at entry, so it is ours)
      to = 0;
      while (!s_axi_rvalid) begin
        @(negedge clk);
        to++;
        if (to > 100000) begin
          errors++; $display("FAIL axi_rd: rvalid timeout rd %h", addr); $finish;
        end
      end
      data = s_axi_rdata;
      @(negedge clk);                         // let the r pulse clear before next txn
    end
  endtask

  // ------------------------------------------------------------------
  // drivers / checkers
  // ------------------------------------------------------------------
  // copy the REAL directory entry into the shadow (hierarchical probe; the
  // r3 read port belongs to the control plane in this bench)
  task automatic peek_dir(input int q, input int blk);
    sdir[q*4096 + blk] = u_dir.mem[(q << QBLK_AW) + blk];
  endtask

  // deterministic filler written into every pooled block at seed time, so
  // any later install pops known content and check_all sees the same base
  // in the physical and expected arrays
  task automatic pre_pattern(input logic [1:0] tier,
                             input logic [31:0] base, input int seed);
    logic [31:0] w;
    logic [63:0] v;
    if (tier == TIER_L0) begin
      for (int i = 0; i < 64; i++) begin
        w = seed + i * 61;
        u_l0.mem[base + i] = w;
        exp_l0[base + i]   = w;
      end
    end else begin
      for (int i = 0; i < 32; i++) begin
        v = (64'(seed) << 32) | (32'(seed) + i);
        l1_mem[(base >> 3) + i] = v;
        exp_l1[(base >> 3) + i] = v;
      end
    end
  endtask

  // seed one pool block through the register interface (deferred ack) and
  // pre-pattern it
  task automatic seed_pool(input logic [1:0] tier, input logic [31:0] base);
    begin
      axi_wr(A_POOL, (32'(tier) << 30) | base);
      pre_pattern(tier, base, 32'(base) ^ 32'h5A5A0000);
    end
  endtask

  // one packet to every enabled query; enable is sampled at ingress
  task automatic send_pkt_all(input flow_key_t key, input int id);
    bit en [QNUM];
    begin
      for (int q = 0; q < QNUM; q++) en[q] = qd[q].enable;
      @(negedge clk);
      ing_val        = 1'b1;
      ing_pkt.key    = key;
      ing_pkt.pkt_id = id;
      ing_pkt.tstamp = 0;
      ing_pkt.rsv    = '0;
      while (!ing_rdy) @(negedge clk);
      @(negedge clk);
      ing_val = 1'b0;
      while (!(wo_val && !wo_is_token)) @(negedge clk);
      for (int q = 0; q < QNUM; q++)
        if (en[q]) ref_apply(q, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
  endtask

  task automatic check_ctr(input int q, input int c, input int exp,
                           input string what);
    logic [31:0] got;
    begin
      @(negedge clk);
      got = u_exec.ctr[q][c];
      if (got !== 32'(exp)) begin
        errors++;
        $display("FAIL %s q%0d c%0d: got %0d exp %0d", what, q, c, got, exp);
      end
    end
  endtask

  task automatic check_ctr_all(input int q, input string tag);
    check_ctr(q, 0, r_att[q], tag);
    check_ctr(q, 1, r_rej[q], tag);
    check_ctr(q, 2, r_att[q] - r_rej[q] - r_dir[q], tag);
    check_ctr(q, 3, r_dir[q], tag);
    check_ctr(q, 4, r_rep[q], tag);
    check_ctr(q, 5, r_sat[q], tag);
  endtask

  task automatic check_all(input string tag);
    logic [31:0] e32;
    logic [63:0] e64;
    begin
      for (int i = 0; i < L0_WORDS; i++) begin
        e32 = exp_l0.exists(i) ? exp_l0[i] : 32'h0;
        if (u_l0.mem[i] !== e32) begin
          errors++;
          $display("FAIL %s L0[%0d]: got %h exp %h", tag, i, u_l0.mem[i], e32);
        end
      end
      for (int i = 0; i < L1_LINES; i++) begin
        e64 = exp_l1.exists(i) ? exp_l1[i] : 64'h0;
        if (l1_mem[i] !== e64) begin
          errors++;
          $display("FAIL %s L1[%0d]: got %h exp %h", tag, i, l1_mem[i], e64);
        end
      end
    end
  endtask

  // ------------------------------------------------------------------
  // shadow model of the tier-manager free stacks: tracks every push/pop
  // (AXI seed, install pop, rollback/remove return, engine issue, release)
  // so installs can be checked against exact LIFO addresses
  // ------------------------------------------------------------------
  logic [31:0] sh_l0 [0:255];
  logic [31:0] sh_l1 [0:255];
  int sh_n_l0 = 0, sh_n_l1 = 0;

  wire rel_l0e = rel_val && (rel_tier == TIER_L0);
  wire rel_l1e = rel_val && (rel_tier == TIER_L1);
  wire cpp_l0  = cp_push_val && (cp_push_tier == TIER_L0);
  wire cpp_l1  = cp_push_val && (cp_push_tier == TIER_L1);
  wire cpn_l0  = cp_pop_val && tm_pop_rdy && (cp_pop_tier == TIER_L0);
  wire cpn_l1  = cp_pop_val && tm_pop_rdy && (cp_pop_tier == TIER_L1);
  wire enn_l0  = mc_val_gated && mc_rdy && (mc_cmd.op == MIG_PROMOTE);
  wire enn_l1  = mc_val_gated && mc_rdy && (mc_cmd.op == MIG_DEMOTE);
  wire sh_pop_l0 = cpn_l0 || enn_l0;
  wire sh_pop_l1 = cpn_l1 || enn_l1;
  wire sh_push_l0 = rel_l0e || cpp_l0;
  wire sh_push_l1 = rel_l1e || cpp_l1;

  always @(posedge clk) begin
    if (cp_flush) begin
      sh_n_l0 <= 0;
      sh_n_l1 <= 0;
    end else begin
      case ({sh_pop_l0, sh_push_l0})
        2'b10: sh_n_l0 <= sh_n_l0 - 1;
        2'b01: begin
          sh_l0[sh_n_l0] <= rel_l0e ? rel_addr : cp_push_addr;
          sh_n_l0 <= sh_n_l0 + 1;
        end
        2'b11: sh_l0[sh_n_l0 - 1] <= rel_l0e ? rel_addr : cp_push_addr;
      endcase
      case ({sh_pop_l1, sh_push_l1})
        2'b10: sh_n_l1 <= sh_n_l1 - 1;
        2'b01: begin
          sh_l1[sh_n_l1] <= rel_l1e ? rel_addr : cp_push_addr;
          sh_n_l1 <= sh_n_l1 + 1;
        end
        2'b11: sh_l1[sh_n_l1 - 1] <= rel_l1e ? rel_addr : cp_push_addr;
      endcase
    end
  end

  task automatic check_pools(input string tag);
    begin
      if ((sh_n_l0 != l0_free_cnt) || (sh_n_l1 != l1_free_cnt)) begin
        errors++;
        $display("FAIL %s pools: shadow l0 %0d vs %0d, l1 %0d vs %0d",
                 tag, sh_n_l0, l0_free_cnt, sh_n_l1, l1_free_cnt);
      end
    end
  endtask

  task automatic rd_pool(output int l0n, output int l1n);
    logic [31:0] d;
    begin
      axi_rd(A_POOL, d);
      l0n = d[12:0];
      l1n = d[28:16];
    end
  endtask

  // ------------------------------------------------------------------
  // migration bookkeeping (upgrade over tb_tier_mgr):
  //   * the engine's directory CAS port (w0) is monitored, so the shadow
  //     directory tracks freeze / replaying / commit in real time;
  //   * the block copy is mirrored at the FIRST REPLAY HANDSHAKE (the
  //     engine only starts replaying after the copy completes), or at the
  //     commit CAS for a migration with zero diverted packets.
  // ------------------------------------------------------------------
  dirent_t    mig_src;
  logic [1:0] mig_dst_tier;
  logic [31:0] mig_dst_addr;
  bit         mig_mirror_done = 1'b1;
  bit         mig_pend = 1'b0;
  logic [DIR_IDXW-1:0] mig_pend_idx;

  task automatic mirror_copy();
    begin
      for (int w = 0; w < 64; w++)
        wr_word32(mig_dst_tier, mig_dst_addr, w,
                  rd_word32(mig_src.tier, mig_src.addr, w));
      mig_mirror_done = 1'b1;
    end
  endtask

  // replayed token latch (applied at its completion, in order)
  token_t      pend_tok;
  logic        pend_gok;
  logic [2:0]  pend_oq;
  logic [15:0] pend_ob;
  logic [31:0] pend_oa;
  logic [1:0]  pend_ot;

  // engine release monitor
  int          rel_seen = 0;
  logic [1:0]  rel_tier_q;
  logic [31:0] rel_addr_q;
  always @(posedge clk) begin
    if (rel_val) begin
      rel_seen   <= rel_seen + 1;
      rel_tier_q <= rel_tier;
      rel_addr_q <= rel_addr;
    end
  end

  always @(posedge clk) begin
    // command accepted by the engine (through the cmd_hold gate)
    if (mc_val_gated && mc_rdy) begin
      mig_src       = sdir_get(mc_cmd.qid, mc_cmd.block);
      mig_dst_tier  = (mc_cmd.op == MIG_PROMOTE) ? TIER_L0 : TIER_L1;
      mig_dst_addr  = mc_cmd.dest_addr;
      mig_mirror_done = 1'b0;
      mig_pend      = 1'b1;
      mig_pend_idx  = {mc_cmd.qid, mc_cmd.block[QBLK_AW-1:0]};
    end
    // engine CAS observed: keep the shadow exact; mirror zero-replay
    // migrations at the commit publish
    if (g_dw_val && g_dw_rdy) begin
      sdir[g_dw_idx[DIR_IDXW-1:QBLK_AW] * 4096 + g_dw_idx[QBLK_AW-1:0]]
        = g_dw_ent;
      if (mig_pend && (g_dw_idx == mig_pend_idx) && g_dw_ent.valid &&
          (g_dw_ent.owner == OWN_RESIDENT)) begin
        if (!mig_mirror_done) mirror_copy();
        mig_pend = 1'b0;
      end
    end
    // first replay handshake: copy complete, mirror before the token's
    // ref_apply lands (its completion comes later)
    if (rp_val && rp_rdy) begin
      if (mig_pend && !mig_mirror_done) mirror_copy();
      pend_tok <= rp_tok;
      pend_gok <= rp_gen_ok;
      pend_oq  <= rp_ovr_qid;
      pend_ob  <= rp_ovr_blk;
      pend_oa  <= rp_ovr_addr;
      pend_ot  <= rp_ovr_tier;
    end
    if (wo_val && wo_is_token) begin
      if (!mig_mirror_done) mirror_copy();   // unreachable backstop
      if (pend_gok)
        ref_apply(pend_tok.qid, pend_tok.key, 1'b1,
                  pend_oq, pend_ob, pend_oa, pend_ot);
    end
  end

  task automatic wait_done(input string tag);
    int t;
    begin
      t = 0;
      while (!mig_busy && t < 100) begin @(negedge clk); t++; end
      t = 0;
      while (mig_busy && t < 200000) begin @(negedge clk); t++; end
      if (mig_busy) begin
        errors++;
        $display("FAIL %s: migration timeout", tag);
      end
      if (!mig_mirror_done) mirror_copy();
    end
  endtask

  // ------------------------------------------------------------------
  // register-interface drivers
  // ------------------------------------------------------------------
  task automatic install_query(input int q, input qdesc_t D,
                               input int nblocks, input logic [1:0] tier,
                               input bit expect_err);
    logic [31:0] st;
    int t;
    begin
      axi_wr(A_QSEL, 32'(q));
      axi_wr(A_QD0,  (32'(D.kind) << 24) | (32'(D.rows) << 12) |
                     (32'(D.cols_log2) << 5) | 32'(D.width));
      axi_wr(A_QS0, D.seed0);
      axi_wr(A_QS1, D.seed1);
      axi_wr(A_QS2, D.seed2);
      axi_wr(A_QS3, D.seed3);
      axi_wr(A_QDHI, (32'(D.gen) << 24) | (32'(D.cand_log2) << 14) |
                     (32'(D.blm_k) << 10));
      axi_wr(A_QCFG, ((tier == TIER_L1) ? 32'h100 : 32'h0) | 32'(nblocks));
      axi_wr(A_QCMD, 32'h1);            // commit (enable stays 0)
      axi_wr(A_QCMD, 32'h2);            // install
      t = 0;
      forever begin
        axi_rd(A_STATUS, st);
        if (st[1] || st[3]) break;
        if (t++ > 20000) begin
          errors++;
          $display("FAIL install q%0d: no install_done/op_err within poll", q);
          break;
        end
      end
      if (expect_err ? !st[3] : (st[3] || !st[1])) begin
        errors++;
        $display("FAIL install q%0d: done=%0b err=%0b (expect_err=%0b)",
                 q, st[1], st[3], expect_err);
      end
      axi_wr(A_STATUS, 32'hA);          // W1C install_done + op_err
    end
  endtask

  task automatic remove_query(input int q);
    logic [31:0] st;
    int t;
    begin
      axi_wr(A_QSEL, 32'(q));
      axi_wr(A_QCMD, 32'h4);            // remove
      t = 0;
      forever begin
        axi_rd(A_STATUS, st);
        if (st[2] || st[3]) break;
        if (t++ > 200000) begin
          errors++;
          $display("FAIL remove q%0d: no remove_done within poll", q);
          break;
        end
      end
      if (st[3]) begin
        errors++;
        $display("FAIL remove q%0d: op_err set", q);
      end
      axi_wr(A_STATUS, 32'hC);          // W1C remove_done + op_err
    end
  endtask

  task automatic read_dir(input int q, input int blk, output dirent_t e);
    logic [31:0] lo, hi;
    begin
      axi_wr(A_DIRIDX, 32'(q) | (32'(blk) << 3));
      axi_rd(A_DENTLO, lo);
      axi_rd(A_DENTHI, hi);
      e = '0;
      e.valid = hi[31];
      e.tier  = hi[30:29];
      e.owner = hi[28:27];
      e.gen   = hi[26:19];
      e.mslot = hi[18:15];
      e.addr  = {hi[14:0], lo[31:15]};
    end
  endtask

  // L0: addr = word address | L1: addr = byte address
  task automatic readout(input logic [1:0] tier, input logic [31:0] addr,
                         output logic [31:0] lo, output logic [31:0] hi);
    logic [31:0] cmdw;
    begin
      if (tier == TIER_L0) cmdw = 32'h1 | (32'(addr) << 5);
      else                 cmdw = 32'h11 | ((addr >> 3) << 5);
      axi_wr(A_ROCMD, cmdw);            // deferred ack: data valid at return
      axi_rd(A_ROLO, lo);
      axi_rd(A_ROHI, hi);
    end
  endtask

  // snapshot + full comparison against the live counters (caller must
  // ensure traffic is idle so nothing moves between snap and compare)
  task automatic tlm_check(input string tag);
    logic [31:0] v;
    begin
      axi_wr(A_TLM, 32'h1);
      for (int q = 0; q < QNUM; q++)
        for (int c = 0; c < 6; c++) begin
          axi_rd(12'h050 + 12'((q * 6 + c) * 4), v);
          if (v !== u_exec.ctr[q][c]) begin
            errors++;
            $display("FAIL %s TLM_Q q%0d c%0d: got %0d hw %0d",
                     tag, q, c, v, u_exec.ctr[q][c]);
          end
        end
      axi_rd(12'h0B0, v);
      if (v !== mig_cnt)     begin errors++; $display("FAIL %s TLM_G mig", tag); end
      axi_rd(12'h0B4, v);
      if (v !== rep_ok_cnt)  begin errors++; $display("FAIL %s TLM_G rep_ok", tag); end
      axi_rd(12'h0B8, v);
      if (v !== rep_drop_cnt) begin errors++; $display("FAIL %s TLM_G rep_drop", tag); end
      axi_rd(12'h0BC, v);
      if (v !== cmd_rej_cnt) begin errors++; $display("FAIL %s TLM_G cmd_rej", tag); end
      axi_rd(12'h0C0, v);
      if (v !== err_cnt)     begin errors++; $display("FAIL %s TLM_G err", tag); end
      axi_rd(12'h0C4, v);
      if (v !== prom_cnt)    begin errors++; $display("FAIL %s TLM_G prom", tag); end
      axi_rd(12'h0C8, v);
      if (v !== dem_cnt)     begin errors++; $display("FAIL %s TLM_G dem", tag); end
      axi_rd(12'h0CC, v);
      if (v !== scan_passes) begin errors++; $display("FAIL %s TLM_G scan_passes", tag); end
      axi_rd(12'h0D0, v);
      if (v !== l0_free_cnt) begin errors++; $display("FAIL %s TLM_G l0_free", tag); end
      axi_rd(12'h0D4, v);
      if (v !== l1_free_cnt) begin errors++; $display("FAIL %s TLM_G l1_free", tag); end
    end
  endtask

  // ------------------------------------------------------------------
  // stimulus
  // ------------------------------------------------------------------
  qdesc_t D;
  dirent_t e, e0, e1;
  flow_key_t key;
  int i, t;
  int l0n, l1n;
  int pre_l1_n, pre_l0_n;
  int att0_pre;
  int dir1_0, rok1_0, rdrop1_0, prom0;
  int q1base, line, byte_;
  bit cp4_pause;
  logic [31:0] st, lo, hi;

  initial begin
    ing_val = 0; ing_pkt = '0;
    m_awvalid = 0; m_wvalid = 0; m_arvalid = 0;
    m_awaddr = 0; m_araddr = 0; m_wdata = 0;
    cp4_pause = 1'b0;
    for (int q = 0; q < QNUM; q++) begin
      r_att[q] = 0; r_rej[q] = 0; r_dir[q] = 0; r_rep[q] = 0; r_sat[q] = 0;
    end

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ================================================================
    // CP1: reset state, write-readback, W-before-AW, unmapped, flush
    // ================================================================
    axi_rd(A_STATUS, st);
    if (st !== 32'h200) begin
      errors++;
      $display("FAIL CP1 STATUS: got %h exp 00000200 (exec_idle only)", st);
    end
    axi_rd(A_CTRL, st);
    if (st !== 32'h1) begin
      errors++;
      $display("FAIL CP1 CTRL default: got %h exp 1 (scan on)", st);
    end
    axi_wr(A_CTRL, 32'h0);              // scan off for the install phases
    axi_rd(A_CTRL, st);
    if (st !== 32'h0) begin
      errors++; $display("FAIL CP1 CTRL scan-off readback: %h", st);
    end
    axi_wr(A_QSEL, 32'd5);
    axi_rd(A_QSEL, st);
    if (st !== 32'd5) begin
      errors++; $display("FAIL CP1 QSEL readback: %0d", st);
    end
    axi_wr(A_QSEL, 32'd3, 1'b1);        // W accepted before AW
    axi_rd(A_QSEL, st);
    if (st !== 32'd3) begin
      errors++; $display("FAIL CP1 QSEL wfirst readback: %0d", st);
    end
    axi_rd(12'h0FC, st);
    if (st !== 32'h0) begin
      errors++; $display("FAIL CP1 unmapped read: %h", st);
    end
    rd_pool(l0n, l1n);
    if (l0n != 0 || l1n != 0) begin
      errors++; $display("FAIL CP1 pool empty: l0 %0d l1 %0d", l0n, l1n);
    end
    // seed one slot, then flush it away (accept path of the flush)
    axi_wr(A_POOL, (32'(TIER_L0) << 30) | 32'(16 * 64));
    rd_pool(l0n, l1n);
    if (l0n != 1) begin
      errors++; $display("FAIL CP1 seed: l0 %0d exp 1", l0n);
    end
    axi_wr(A_CTRL, 32'h2);              // flush pulse (scan stays off)
    rd_pool(l0n, l1n);
    if (l0n != 0) begin
      errors++; $display("FAIL CP1 flush: l0 %0d exp 0", l0n);
    end
    check_pools("CP1");
    $display("CP1 register basics done");

    // ================================================================
    // CP2: pool seeding through POOL_PUSH + telemetry snapshot
    // ================================================================
    for (int k = 0; k < 4; k++) seed_pool(TIER_L0, 32'((16 + k) * 64));
    for (int k = 0; k < 4; k++) seed_pool(TIER_L1, 32'((8 + k) * 256));
    rd_pool(l0n, l1n);
    if (l0n != 4 || l1n != 4) begin
      errors++;
      $display("FAIL CP2 pools: l0 %0d l1 %0d exp 4/4", l0n, l1n);
    end
    check_pools("CP2");
    tlm_check("CP2");
    // r3 read of a never-installed entry: all-zero
    read_dir(0, 0, e);
    if (e !== '0) begin
      errors++; $display("FAIL CP2 invalid dirent: %h/%h", e.addr, e);
    end
    $display("CP2 pool seed + telemetry done");

    // ================================================================
    // CP3: install q0 (2-block L1 CMS) and drive traffic through it
    // ================================================================
    D = '0;
    D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 8; D.width = WID_8;
    D.seed0 = 32'hDEADBEEF; D.seed1 = 32'h12345678;
    D.seed2 = 32'h00112233; D.seed3 = 32'h44556677; D.gen = 8'd5;
    pre_l1_n = sh_n_l1;
    install_query(0, D, 2, TIER_L1, 1'b0);
    read_dir(0, 0, e0);
    read_dir(0, 1, e1);
    peek_dir(0, 0); peek_dir(0, 1);
    if (!e0.valid || e0.tier != TIER_L1 || e0.owner != OWN_RESIDENT ||
        e0.gen != 8'd5 || e0.mslot != 0 ||
        e0.addr != sh_l1[pre_l1_n - 1] || e0 !== sdir_get(0, 0)) begin
      errors++;
      $display("FAIL CP3 e0: v%b t%0d o%0d g%0d a%h (exp a%h)",
               e0.valid, e0.tier, e0.owner, e0.gen, e0.addr,
               sh_l1[pre_l1_n - 1]);
    end
    if (!e1.valid || e1.tier != TIER_L1 || e1.owner != OWN_RESIDENT ||
        e1.gen != 8'd5 || e1.addr != sh_l1[pre_l1_n - 2] ||
        e1 !== sdir_get(0, 1)) begin
      errors++;
      $display("FAIL CP3 e1: v%b a%h (exp a%h)",
               e1.valid, e1.addr, sh_l1[pre_l1_n - 2]);
    end
    if (!qd[0].enable || qd[0].kind != KIND_CMS || qd[0].rows != 2 ||
        qd[0].seed0 != 32'hDEADBEEF || qgen[0] != 8'd5) begin
      errors++;
      $display("FAIL CP3 qd0 publish: en=%b kind=%0d rows=%0d gen=%0d",
               qd[0].enable, qd[0].kind, qd[0].rows, qgen[0]);
    end
    rd_pool(l0n, l1n);
    if (l0n != 4 || l1n != 2) begin
      errors++;
      $display("FAIL CP3 pools: l0 %0d l1 %0d exp 4/2", l0n, l1n);
    end
    check_pools("CP3");
    for (i = 0; i < 50; i++)
      send_pkt_all(gen_key(1000 + i), 1000 + i);
    check_ctr_all(0, "CP3");
    tlm_check("CP3");
    check_all("CP3");
    $display("CP3 install + traffic done (att=%0d)", r_att[0]);

    // ================================================================
    // CP4: ONLINE install of q1 while q0 traffic flows.  The pump pauses
    // only around the publish edge: a packet straddling enable 0->1 could
    // not be modelled (hardware samples enable per query mid-packet).
    // ================================================================
    D = '0;
    D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 7; D.width = WID_8;
    D.seed0 = 32'hA1B2C3D4; D.seed1 = 32'hB2C3D4E5;
    D.seed2 = 32'hC3D4E5F6; D.seed3 = 32'hD4E5F607; D.gen = 8'd3;
    pre_l1_n = sh_n_l1;
    fork
      begin : cp4_pump
        for (int j = 0; j < 40; j++) begin
          while (cp4_pause) @(negedge clk);
          send_pkt_all(gen_key(5000 + j), 5000 + j);
        end
      end
      begin : cp4_inst
        axi_wr(A_QSEL, 32'd1);
        axi_wr(A_QD0,  (32'(D.kind) << 24) | (32'(D.rows) << 12) |
                       (32'(D.cols_log2) << 5) | 32'(D.width));
        axi_wr(A_QS0, D.seed0); axi_wr(A_QS1, D.seed1);
        axi_wr(A_QS2, D.seed2); axi_wr(A_QS3, D.seed3);
        axi_wr(A_QDHI, 32'(D.gen) << 24);
        axi_wr(A_QCFG, 32'h100 | 32'd1);       // L1, 1 block
        axi_wr(A_QCMD, 32'h1);                 // commit, traffic still flowing
        cp4_pause = 1'b1;                      // gate the publish edge
        while (!exec_idle) @(negedge clk);
        repeat (2) @(negedge clk);
        axi_wr(A_QCMD, 32'h2);                 // install
        t = 0;
        forever begin
          axi_rd(A_STATUS, st);
          if (st[1] || st[3]) break;
          if (t++ > 20000) begin
            errors++; $display("FAIL CP4 install timeout"); break;
          end
        end
        if (!st[1]) begin errors++; $display("FAIL CP4 install: no done"); end
        axi_wr(A_STATUS, 32'h2);               // W1C
        // sample the freshly-installed entry into the shadow BEFORE the pump
        // resumes: packets applied for q1 after this point must find a valid
        // shadow entry, otherwise the reference model tallies them as rejects
        peek_dir(1, 0);
        cp4_pause = 1'b0;
      end
    join
    read_dir(1, 0, e);
    peek_dir(1, 0);
    if (!e.valid || e.tier != TIER_L1 || e.owner != OWN_RESIDENT ||
        e.gen != 8'd3 || e.addr != sh_l1[pre_l1_n - 1] ||
        e !== sdir_get(1, 0)) begin
      errors++;
      $display("FAIL CP4 e_q1: v%b t%0d g%0d a%h (exp a%h)",
               e.valid, e.tier, e.gen, e.addr, sh_l1[pre_l1_n - 1]);
    end
    // q0 untouched by the q1 install
    peek_dir(0, 0); peek_dir(0, 1);
    if (sdir_get(0, 0) !== e0 || sdir_get(0, 1) !== e1) begin
      errors++; $display("FAIL CP4 q0 dir polluted by install");
    end
    check_ctr_all(0, "CP4");
    check_ctr_all(1, "CP4");
    rd_pool(l0n, l1n);
    if (l0n != 4 || l1n != 1) begin
      errors++;
      $display("FAIL CP4 pools: l0 %0d l1 %0d exp 4/1", l0n, l1n);
    end
    check_pools("CP4");
    check_all("CP4");
    $display("CP4 online install done (att0=%0d att1=%0d)",
             r_att[0], r_att[1]);

    // ================================================================
    // CP5: remove q0 online (q1 stays live), then verify silence
    // ================================================================
    att0_pre = u_exec.ctr[0][0];
    remove_query(0);
    for (int b = 0; b < 16; b++) begin
      read_dir(0, b, e);
      if (e.valid) begin
        errors++; $display("FAIL CP5 q0 blk%0d still valid", b);
      end
    end
    peek_dir(0, 0);
    if (qd[0].enable) begin
      errors++; $display("FAIL CP5 q0 still enabled");
    end
    if (qgen[0] != 8'd6) begin
      errors++; $display("FAIL CP5 q0 gen: %0d exp 6", qgen[0]);
    end
    rd_pool(l0n, l1n);
    if (l0n != 4 || l1n != 3) begin
      errors++;
      $display("FAIL CP5 pools: l0 %0d l1 %0d exp 4/3", l0n, l1n);
    end
    // blocks returned LIFO: b1 (last walked) on top
    if (sh_n_l1 != 3 || sh_l1[2] != e1.addr || sh_l1[1] != e0.addr) begin
      errors++;
      $display("FAIL CP5 return order: [%h %h] top2 %h %h",
               e0.addr, e1.addr, sh_l1[1], sh_l1[2]);
    end
    check_pools("CP5");
    // removed query is silent: packets skip it (att flat, not rejected)
    for (i = 0; i < 10; i++)
      send_pkt_all(gen_key(6000 + i), 6000 + i);
    check_ctr(0, 0, att0_pre, "CP5 att flat");
    check_ctr_all(0, "CP5");
    check_ctr_all(1, "CP5");
    check_all("CP5");
    $display("CP5 remove done (att0 flat at %0d, att1=%0d)",
             att0_pre, r_att[1]);

    // ================================================================
    // CP6: sketch readout through the engine-port bypasses
    // ================================================================
    // load q1's block with 300 more updates (also builds heat for CP8)
    for (i = 0; i < 300; i++)
      send_pkt_all(gen_key(7000 + i), 7000 + i);
    check_ctr_all(1, "CP6a");
    check_all("CP6a");
    peek_dir(1, 0);
    q1base = sdir_get(1, 0).addr;
    line   = (q1base >> 3) + 7;
    byte_  = line << 3;
    readout(TIER_L1, 32'(byte_), lo, hi);
    if ({hi, lo} !== l1_mem[line]) begin
      errors++;
      $display("FAIL CP6 L1 readout line %0d: got %h_%h exp %h",
               line, hi, lo, l1_mem[line]);
    end
    // an L0 query: install, drive, read out, remove (pool return path)
    D = '0;
    D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 7; D.width = WID_8;
    D.seed0 = 32'h0F1E2D3C; D.seed1 = 32'h1E2D3C4B;
    D.seed2 = 32'h2D3C4B5A; D.seed3 = 32'h3C4B5A69; D.gen = 8'd9;
    pre_l0_n = sh_n_l0;
    install_query(2, D, 1, TIER_L0, 1'b0);
    read_dir(2, 0, e);
    peek_dir(2, 0);
    if (!e.valid || e.tier != TIER_L0 || e.owner != OWN_RESIDENT ||
        e.gen != 8'd9 || e.addr != sh_l0[pre_l0_n - 1] ||
        e !== sdir_get(2, 0)) begin
      errors++;
      $display("FAIL CP6 e_q2: v%b t%0d g%0d a%h (exp a%h)",
               e.valid, e.tier, e.gen, e.addr, sh_l0[pre_l0_n - 1]);
    end
    rd_pool(l0n, l1n);
    if (l0n != 3 || l1n != 3) begin
      errors++;
      $display("FAIL CP6 pools post-q2: l0 %0d l1 %0d exp 3/3", l0n, l1n);
    end
    for (i = 0; i < 30; i++)
      send_pkt_all(gen_key(8000 + i), 8000 + i);
    check_ctr_all(1, "CP6b");
    check_ctr_all(2, "CP6b");
    check_all("CP6b");
    readout(TIER_L0, sh_l0[pre_l0_n - 1] + 32'd10, lo, hi);
    if (lo !== u_l0.mem[sh_l0[pre_l0_n - 1] + 10] || hi !== 32'h0) begin
      errors++;
      $display("FAIL CP6 L0 readout: got %h exp %h",
               lo, u_l0.mem[sh_l0[pre_l0_n - 1] + 10]);
    end
    // the bypass reads any address (no ownership check in v1): a pooled L0
    // block must read back its seeded pattern
    readout(TIER_L0, 32'(16 * 64 + 5), lo, hi);
    if (lo !== u_l0.mem[16 * 64 + 5] || hi !== 32'h0) begin
      errors++;
      $display("FAIL CP6 L0 pool readout: got %h exp %h",
               lo, u_l0.mem[16 * 64 + 5]);
    end
    remove_query(2);
    read_dir(2, 0, e);
    if (e.valid) begin errors++; $display("FAIL CP6 q2 not invalid"); end
    if (qgen[2] != 8'd10) begin
      errors++; $display("FAIL CP6 q2 gen: %0d exp 10", qgen[2]);
    end
    rd_pool(l0n, l1n);
    if (l0n != 4 || l1n != 3) begin
      errors++;
      $display("FAIL CP6 pools post-rm: l0 %0d l1 %0d exp 4/3", l0n, l1n);
    end
    check_pools("CP6");
    check_all("CP6c");
    $display("CP6 readout done (L1 line %0d, L0 base %0d)",
             line, sh_l0[pre_l0_n - 1]);

    // ================================================================
    // CP7: quota rollback (5 blocks asked, 4 in the pool), then recovery
    // ================================================================
    D = '0;
    D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 7; D.width = WID_8;
    D.seed0 = 32'h99999999; D.seed1 = 32'h88888888;
    D.seed2 = 32'h77777777; D.seed3 = 32'h66666666; D.gen = 8'd11;
    pre_l0_n = sh_n_l0;
    install_query(3, D, 5, TIER_L0, 1'b1);    // expect op_err rollback
    for (int b = 0; b < 16; b++) begin
      peek_dir(3, b);
      if (sdir_get(3, b).valid) begin
        errors++; $display("FAIL CP7 q3 blk%0d residual valid", b);
      end
    end
    read_dir(3, 0, e);
    if (e.valid) begin errors++; $display("FAIL CP7 r3 readback valid"); end
    if (qd[3].enable) begin errors++; $display("FAIL CP7 q3 enabled"); end
    rd_pool(l0n, l1n);
    if (l0n != 4 || l1n != 3) begin
      errors++;
      $display("FAIL CP7 pools post-rollback: l0 %0d l1 %0d exp 4/3", l0n, l1n);
    end
    // LIFO order fully restored by the rollback pushes
    if (sh_n_l0 != 4 || sh_l0[0] != 32'(16 * 64) ||
        sh_l0[1] != 32'(17 * 64) || sh_l0[2] != 32'(18 * 64) ||
        sh_l0[3] != 32'(19 * 64)) begin
      errors++;
      $display("FAIL CP7 stack order: %h %h %h %h",
               sh_l0[0], sh_l0[1], sh_l0[2], sh_l0[3]);
    end
    axi_rd(A_STATUS, st);
    if (st[3]) begin errors++; $display("FAIL CP7 op_err not W1C'd"); end
    check_pools("CP7");
    check_all("CP7a");
    // recovery: a 1-block install on the same query succeeds
    install_query(3, D, 1, TIER_L0, 1'b0);
    read_dir(3, 0, e);
    peek_dir(3, 0);
    if (!e.valid || e.tier != TIER_L0 || e.gen != 8'd11 ||
        e.addr != 32'(19 * 64) || e !== sdir_get(3, 0)) begin
      errors++;
      $display("FAIL CP7 recovery e: v%b t%0d g%0d a%h",
               e.valid, e.tier, e.gen, e.addr);
    end
    rd_pool(l0n, l1n);
    if (l0n != 3) begin errors++; $display("FAIL CP7 pools post-recovery l0 %0d", l0n); end
    remove_query(3);
    rd_pool(l0n, l1n);
    if (l0n != 4) begin errors++; $display("FAIL CP7 pools post-rm l0 %0d", l0n); end
    check_pools("CP7");
    check_all("CP7b");
    $display("CP7 rollback + recovery done");

    // ================================================================
    // CP8: scan re-enable, promote with mid-migration readout, 100-packet
    // coexistence, telemetry consistency
    // ================================================================
    // only q1 is installed: its block sits in L1 with 300+ packets of heat
    axi_wr(A_CTRL, 32'h1);              // scan on
    axi_rd(A_CTRL, st);
    if (st !== 32'h1) begin errors++; $display("FAIL CP8 scan on: %h", st); end
    prom0 = prom_cnt;
    t = 0;
    while (prom_cnt == prom0 && t < 8000) begin @(negedge clk); t++; end
    if (prom_cnt != prom0 + 1) begin
      errors++; $display("FAIL CP8: no promote issued (heat %0d)",
                         u_heat.mem[(1 << QBLK_AW) * 1]);
    end
    dir1_0   = r_dir[1];
    rok1_0   = rep_ok_cnt;
    rdrop1_0 = rep_drop_cnt;
    // traffic + readout interleaved with the in-flight migration: the
    // readout parks on mig_busy under cmd_hold and returns post-commit data
    for (i = 0; i < 15; i++) begin
      send_pkt_all(gen_key(9000 + i), 9000 + i);
      peek_dir(1, 0);
      if (i == 4) begin
        readout(TIER_L0, mig_dst_addr + 32'd3, lo, hi);
        if (lo !== u_l0.mem[mig_dst_addr + 3] || hi !== 32'h0) begin
          errors++;
          $display("FAIL CP8 busy readout: got %h exp %h",
                   lo, u_l0.mem[mig_dst_addr + 3]);
        end
      end
    end
    wait_done("CP8a");
    // conservation AFTER the drain: the last diverted tokens are still in
    // flight right after the loop, but the engine replays everything (and
    // observes every completion) before its commit clears mig_busy
    if ((r_dir[1] - dir1_0) != (rep_ok_cnt - rok1_0)) begin
      errors++;
      $display("FAIL CP8 conservation: div %0d rep %0d",
               r_dir[1] - dir1_0, rep_ok_cnt - rok1_0);
    end
    if (rep_drop_cnt != rdrop1_0) begin
      errors++; $display("FAIL CP8 drops: %0d", rep_drop_cnt - rdrop1_0);
    end
    peek_dir(1, 0);
    e = sdir_get(1, 0);
    if (!e.valid || e.owner != OWN_RESIDENT || e.tier != TIER_L0 ||
        e.addr != mig_dst_addr) begin
      errors++;
      $display("FAIL CP8 post-promote: v%b o%0d t%0d a%h exp a%h",
               e.valid, e.owner, e.tier, e.addr, mig_dst_addr);
    end
    rd_pool(l0n, l1n);
    if (l0n != 3 || l1n != 4) begin
      errors++;
      $display("FAIL CP8 pools post-promote: l0 %0d l1 %0d exp 3/4", l0n, l1n);
    end
    check_pools("CP8a");
    check_all("CP8a");
    // 100 packets with the scan live (further migrations allowed); poll
    // live telemetry mid-run to prove the registers never hang
    for (i = 0; i < 100; i++) begin
      send_pkt_all(gen_key(10000 + i), 10000 + i);
      peek_dir(1, 0);
      if (i % 25 == 24) begin
        rd_pool(l0n, l1n);
        axi_rd(A_STATUS, st);
        if ((^st) === 1'bx) begin
          errors++; $display("FAIL CP8 STATUS read X at pkt %0d", i);
        end
      end
    end
    // quiesce: scan off, drain any in-flight migration
    axi_wr(A_CTRL, 32'h0);
    t = 0;
    while ((mig_busy || mc_val_raw) && t < 200000) begin @(negedge clk); t++; end
    if (mig_busy) begin errors++; $display("FAIL CP8 drain timeout"); end
    if (!mig_mirror_done) mirror_copy();
    check_all("CP8");
    check_ctr_all(1, "CP8");
    check_pools("CP8");
    tlm_check("CP8");
    if (prom_cnt < 1)   begin errors++; $display("FAIL CP8 prom_cnt %0d", prom_cnt); end
    if (mig_cnt != prom_cnt + dem_cnt) begin
      errors++;
      $display("FAIL CP8 mig %0d != prom %0d + dem %0d",
               mig_cnt, prom_cnt, dem_cnt);
    end
    if (scan_passes == 0) begin errors++; $display("FAIL CP8 scan_passes 0"); end
    if (rep_drop_cnt != 0) begin errors++; $display("FAIL CP8 rep_drop %0d", rep_drop_cnt); end
    if (cmd_rej_cnt != 0) begin errors++; $display("FAIL CP8 cmd_rej %0d", cmd_rej_cnt); end
    if (err_cnt != 0)     begin errors++; $display("FAIL CP8 err %0d", err_cnt); end
    // final cleanup: remove q1 and confirm every pool block came home
    remove_query(1);
    for (int b = 0; b < 16; b++) begin
      read_dir(1, b, e);
      if (e.valid) begin errors++; $display("FAIL CP8-end q1 blk%0d valid", b); end
    end
    rd_pool(l0n, l1n);
    if (l0n + l1n != 8) begin
      errors++;
      $display("FAIL CP8-end pool total: l0 %0d + l1 %0d != 8", l0n, l1n);
    end
    check_pools("CP8-end");
    check_all("CP8-end");
    $display("CP8 scan + promote + coexistence done (prom=%0d dem=%0d mig=%0d)",
             prom_cnt, dem_cnt, mig_cnt);

    // ================================================================
    repeat (4) @(negedge clk);
    if (errors == 0)
      $display("[tb_ctrl_plane] ALL TESTS PASSED");
    else
      $display("[tb_ctrl_plane] %0d ERRORS", errors);
    $finish;
  end

  // safety net
  initial begin
    #50_000_000;
    $display("[tb_ctrl_plane] TIMEOUT");
    $finish;
  end

endmodule
