// Migration engine regression: the full single-writer loop in hardware.
//
// DUT set (the real blocks, no stubs):
//   work_mux -> smu_exec -> state_directory / l0_store / heat_table
//        ^          |  \-> mig_engine (diverts in, replay out)
//        |__________|     mig_engine -> directory CAS, L0/L1 copy
//   smu_exec L1 --\
//                  l1_arb2 -> behavioural L1 memory
//   mig_engine ---/
//
// The reference model is the tb_smu_exec one (byte-exact mirror of the
// SMU update schedule) extended with the migration semantics:
//
//   * the directory shadow is synchronised from the REAL directory (r2
//     port) after every packet -- a freeze/commit CAS can never race an
//     in-flight packet because the engine quiesces the SMU first, so the
//     post-completion shadow is exactly what the packet saw;
//   * the block copy is mirrored into the expected memories lazily, at
//     the first replay completion or at migration end: by then every
//     pre-freeze update is in the shadow and nothing has written the
//     destination yet;
//   * replayed tokens are ref_applied at their wo completion (in-order
//     with packets), so Eq. 1 conservation and exactly-once are checked
//     by the same full-memory compare + counter lockstep as before.

`timescale 1ns/1ps

module tb_mig_engine;

  import sketchsoc_pkg::*;

  localparam int unsigned QBLK_AW  = 4;
  localparam int unsigned L0_AW    = 11;
  localparam int unsigned L0_WORDS = 1 << L0_AW;
  localparam int unsigned L1_LINES = 2048;
  localparam int unsigned DIR_IDXW = QID_W + QBLK_AW;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  int errors = 0;

  // ------------------------------------------------------------------
  // signals
  // ------------------------------------------------------------------
  qdesc_t qd [QNUM];
  logic [7:0] qgen [QNUM];
  always_comb begin
    for (int q = 0; q < QNUM; q++) qgen[q] = qd[q].gen;
  end

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

  // exec heat
  logic                ht_val, ht_rdy;
  logic [DIR_IDXW-1:0] ht_idx;

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

  // engine command / control
  logic     mc_val, mc_rdy;
  mig_cmd_t mc_cmd;
  logic     req_quiesce;
  logic     mig_busy;
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

  // engine L0 (r1/w1)
  logic             g_l0r_val, g_l0r_rdy, g_l0r_dv;
  logic [L0_AW-1:0] g_l0r_addr;
  logic [31:0]      g_l0r_data;
  logic             g_l0w_val, g_l0w_rdy;
  logic [L0_AW-1:0] g_l0w_addr;
  logic [31:0]      g_l0w_data;
  logic [3:0]       g_l0w_strb;

  // engine L1 (arbiter m1)
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

  // arbiter slave side -> L1 model
  logic        s_arvalid, s_arready, s_rvalid, s_rready;
  logic [31:0] s_araddr;
  logic [63:0] s_rdata;
  logic        s_awvalid, s_awready, s_bvalid, s_bready;
  logic [31:0] s_awaddr;
  logic [63:0] s_awdata;
  logic [7:0]  s_awstrb;

  // TB directory r2 + w1 (control-plane role)
  logic                t_dr_val, t_dr_rdy, t_dr_dv;
  logic [DIR_IDXW-1:0] t_dr_idx;
  dirent_t             t_dr_ent;
  logic                t_dw_val, t_dw_rdy;
  logic [DIR_IDXW-1:0] t_dw_idx;
  dirent_t             t_dw_ent;

  // TB heat scan (tied off here; heat is verified in tb_smu_exec)
  logic                h_scn_val, h_scn_rdy, h_scn_dv;
  logic [DIR_IDXW-1:0] h_scn_idx;
  logic [15:0]         h_scn_heat;

  // exec accounting readout
  logic [4:0]  acc_sel;
  logic [31:0] acc_rd;

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
    .req_quiesce(req_quiesce),
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
    .ht_val(ht_val), .ht_rdy(ht_rdy), .ht_idx(ht_idx),
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
    .cmd_val(mc_val), .cmd_rdy(mc_rdy), .cmd(mc_cmd),
    .qgen(qgen),
    .exec_idle(exec_idle), .req_quiesce(req_quiesce),
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

  l1_arb2 u_arb (
    .clk(clk), .rst_n(rst_n),
    .m0_arvalid(e_arvalid), .m0_arready(e_arready), .m0_araddr(e_araddr),
    .m0_rvalid(e_rvalid), .m0_rready(e_rready), .m0_rdata(e_rdata),
    .m0_awvalid(e_awvalid), .m0_awready(e_awready), .m0_awaddr(e_awaddr),
    .m0_awdata(e_awdata), .m0_awstrb(e_awstrb),
    .m0_bvalid(e_bvalid), .m0_bready(e_bready),
    .m1_arvalid(g_arvalid), .m1_arready(g_arready), .m1_araddr(g_araddr),
    .m1_rvalid(g_rvalid), .m1_rready(g_rready), .m1_rdata(g_rdata),
    .m1_awvalid(g_awvalid), .m1_awready(g_awready), .m1_awaddr(g_awaddr),
    .m1_awdata(g_awdata), .m1_awstrb(g_awstrb),
    .m1_bvalid(g_bvalid), .m1_bready(g_bready),
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
    .r1_val(g_l0r_val), .r1_rdy(g_l0r_rdy), .r1_addr(g_l0r_addr),
    .r1_data(g_l0r_data), .r1_dv(g_l0r_dv),
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
    .r3_val(1'b0), .r3_idx('0),          // control-plane port: unused here
    .w0_val(g_dw_val), .w0_rdy(g_dw_rdy), .w0_idx(g_dw_idx),
    .w0_ent(g_dw_ent),
    .w1_val(t_dw_val), .w1_rdy(t_dw_rdy), .w1_idx(t_dw_idx),
    .w1_ent(t_dw_ent)
  );

  heat_table #(.QBLK_AW(QBLK_AW)) u_heat (
    .clk(clk), .rst_n(rst_n),
    .acc_val(ht_val), .acc_rdy(ht_rdy), .acc_idx(ht_idx),
    .scn_val(h_scn_val), .scn_rdy(h_scn_rdy), .scn_idx(h_scn_idx),
    .scn_decay(1'b0), .scn_heat(h_scn_heat), .scn_dv(h_scn_dv)
  );

  // ------------------------------------------------------------------
  // behavioural L1 (same model as tb_smu_exec)
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
  // reference model (identical semantics to tb_smu_exec)
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

  // mirror of the DUT update for one (query, work item) -- same code as
  // tb_smu_exec, minus HH (not exercised here)
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
  // drivers / checkers
  // ------------------------------------------------------------------
  task automatic dir_set(input int q, input int blk, input dirent_t e);
    begin
      @(negedge clk);
      t_dw_val = 1'b1;
      t_dw_idx = (q << QBLK_AW) + blk;
      t_dw_ent = e;
      while (!t_dw_rdy) @(negedge clk);
      @(negedge clk);
      t_dw_val = 1'b0;
      sdir[q*4096 + blk] = e;
    end
  endtask

  // read the REAL directory entry into the shadow (see header: safe
  // because the engine quiesces the SMU around every CAS)
  task automatic sync_sdir(input int q, input int blk);
    begin
      @(negedge clk);
      t_dr_val = 1'b1;
      t_dr_idx = (q << QBLK_AW) + blk;
      while (!t_dr_rdy) @(negedge clk);
      @(negedge clk);
      t_dr_val = 1'b0;
      while (!t_dr_dv) @(negedge clk);
      sdir[q*4096 + blk] = t_dr_ent;
    end
  endtask

  int l0_alloc = 0;
  int l1_alloc = 0;

  task automatic setup_query(input int q, input qdesc_t D,
                             input int nblocks, input logic [1:0] tier);
    dirent_t e;
    logic [31:0] base;
    begin
      qd[q] = D;
      for (int b = 0; b < nblocks; b++) begin
        if (tier == TIER_L0) begin
          base = l0_alloc * 64;
          l0_alloc++;
        end else begin
          base = l1_alloc * 256;
          l1_alloc++;
        end
        e        = '0;
        e.valid  = 1'b1;
        e.tier   = tier;
        e.owner  = OWN_RESIDENT;
        e.gen    = D.gen;
        e.mslot  = 0;
        e.addr   = base;
        dir_set(q, b, e);
      end
    end
  endtask

  task automatic send_pkt(input flow_key_t key, input int id);
    begin
      @(negedge clk);
      ing_val       = 1'b1;
      ing_pkt.key   = key;
      ing_pkt.pkt_id = id;
      ing_pkt.tstamp = 0;
      ing_pkt.rsv   = '0;
      while (!ing_rdy) @(negedge clk);
      @(negedge clk);
      ing_val = 1'b0;
      // in-order completion; replay completions are handled by the monitor
      while (!(wo_val && !wo_is_token)) @(negedge clk);
    end
  endtask

  task automatic check_ctr(input int q, input int c, input int exp,
                           input string what);
    logic [31:0] got;
    begin
      acc_sel = (q << 3) + c;
      @(negedge clk);
      got = acc_rd;
      if (got !== exp[31:0]) begin
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

  task automatic check_eng(input int mig, input int rok, input int rdrop,
                           input int crej, input int err, input string tag);
    begin
      if (mig_cnt !== mig[31:0]) begin
        errors++; $display("FAIL %s mig_cnt: got %0d exp %0d", tag, mig_cnt, mig);
      end
      if (rep_ok_cnt !== rok[31:0]) begin
        errors++; $display("FAIL %s rep_ok: got %0d exp %0d", tag, rep_ok_cnt, rok);
      end
      if (rep_drop_cnt !== rdrop[31:0]) begin
        errors++; $display("FAIL %s rep_drop: got %0d exp %0d", tag, rep_drop_cnt, rdrop);
      end
      if (cmd_rej_cnt !== crej[31:0]) begin
        errors++; $display("FAIL %s cmd_rej: got %0d exp %0d", tag, cmd_rej_cnt, crej);
      end
      if (err_cnt !== err[31:0]) begin
        errors++; $display("FAIL %s err: got %0d exp %0d", tag, err_cnt, err);
      end
    end
  endtask

  // ------------------------------------------------------------------
  // migration bookkeeping
  // ------------------------------------------------------------------
  dirent_t    mig_src;                    // pre-freeze shadow entry
  logic [1:0] mig_dst_tier;
  logic [31:0] mig_dst_addr;
  bit         mig_mirror_done;

  // mirror the block copy into the expected memories (source snapshot ==
  // destination copy; replayed-token writes are applied on top afterwards)
  task automatic mirror_copy();
    logic [31:0] v;
    begin
      for (int w = 0; w < 64; w++) begin
        v = rd_word32(mig_src.tier, mig_src.addr, w);
        wr_word32(mig_dst_tier, mig_dst_addr, w, v);
      end
      mig_mirror_done = 1'b1;
    end
  endtask

  task automatic do_mig(input logic [1:0] op, input int q, input int blk,
                        input logic [31:0] dest);
    begin
      mig_src         = sdir_get(q, blk);
      mig_dst_tier    = (op == MIG_PROMOTE) ? TIER_L0 : TIER_L1;
      mig_dst_addr    = dest;
      mig_mirror_done = 1'b0;
      @(negedge clk);
      mc_val          = 1'b1;
      mc_cmd.op       = op;
      mc_cmd.qid      = q[2:0];
      mc_cmd.block    = blk[15:0];
      mc_cmd.dest_addr = dest;
      while (!mc_rdy) @(negedge clk);
      @(negedge clk);
      mc_val = 1'b0;
    end
  endtask

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

  // release-pulse monitor
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

  // ------------------------------------------------------------------
  // replay monitor: replayed tokens are ref_applied at completion, in
  // order with the packet stream (single outstanding replay token)
  // ------------------------------------------------------------------
  token_t      pend_tok;
  logic        pend_gok;
  logic [2:0]  pend_oq;
  logic [15:0] pend_ob;
  logic [31:0] pend_oa;
  logic [1:0]  pend_ot;

  always @(posedge clk) begin
    if (rp_val && rp_rdy) begin
      pend_tok <= rp_tok;
      pend_gok <= rp_gen_ok;
      pend_oq  <= rp_ovr_qid;
      pend_ob  <= rp_ovr_blk;
      pend_oa  <= rp_ovr_addr;
      pend_ot  <= rp_ovr_tier;
    end
    if (wo_val && wo_is_token) begin
      if (!mig_mirror_done) mirror_copy();
      if (pend_gok)
        ref_apply(pend_tok.qid, pend_tok.key, 1'b1,
                  pend_oq, pend_ob, pend_oa, pend_ot);
    end
  end

  // ------------------------------------------------------------------
  // stimulus
  // ------------------------------------------------------------------
  qdesc_t D;
  dirent_t e;
  flow_key_t key;
  int dir0, rok0, rdrop0;
  int i;

  initial begin
    ing_val = 0; ing_pkt = '0;
    mc_val = 0; mc_cmd = '0;
    t_dr_val = 0; t_dr_idx = 0; t_dw_val = 0; t_dw_idx = 0; t_dw_ent = '0;
    h_scn_val = 0; h_scn_idx = 0;
    acc_sel = 0;
    mig_mirror_done = 1'b1;
    for (int q = 0; q < QNUM; q++) begin
      qd[q] = '0;
      r_att[q] = 0; r_rej[q] = 0; r_dir[q] = 0; r_rep[q] = 0; r_sat[q] = 0;
    end

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ================= M1: demote a cold block, no traffic ============
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 3; D.cols_log2 = 8;
    D.width = WID_8; D.seed0 = 32'h11111111; D.seed1 = 32'h22222222;
    D.seed2 = 32'h33333333; D.seed3 = 32'h44444444; D.gen = 8'd1;
    setup_query(0, D, 3, TIER_L0);
    for (i = 0; i < 40; i++) begin
      key = gen_key(i % 20);
      send_pkt(key, i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("M1a");
    // demote block 1 -> L1
    do_mig(MIG_DEMOTE, 0, 1, l1_alloc * 256);
    l1_alloc++;
    wait_done("M1");
    sync_sdir(0, 1);
    e = sdir_get(0, 1);
    if (!e.valid || e.tier != TIER_L1 || e.owner != OWN_RESIDENT ||
        e.addr != mig_dst_addr || e.gen != D.gen + 1) begin
      errors++;
      $display("FAIL M1 committed entry: %h", e);
    end
    if (rel_seen != 1 || rel_tier_q != TIER_L0 || rel_addr_q != mig_src.addr) begin
      errors++;
      $display("FAIL M1 release: seen=%0d tier=%0d addr=%h",
               rel_seen, rel_tier_q, rel_addr_q);
    end
    check_eng(1, 0, 0, 0, 0, "M1");
    // traffic now lands in L1
    for (i = 0; i < 20; i++) begin
      key = gen_key(100 + i);
      send_pkt(key, 1000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("M1b");
    check_ctr_all(0, "M1b");
    $display("M1 demote-cold done");

    // ================= M2: demote a hot block, diverts + replay =======
    for (i = 0; i < 30; i++) begin
      key = gen_key(200 + (i % 25));
      send_pkt(key, 2000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    dir0   = r_dir[0];
    rok0   = 0;                     // rep_ok delta measured via counter
    rdrop0 = rep_drop_cnt;
    do_mig(MIG_DEMOTE, 0, 2, l1_alloc * 256);
    l1_alloc++;
    for (i = 0; i < 12; i++) begin
      key = gen_key(300 + i);
      send_pkt(key, 2100 + i);
      sync_sdir(0, 2);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    wait_done("M2");
    sync_sdir(0, 2);
    e = sdir_get(0, 2);
    if (!e.valid || e.tier != TIER_L1 || e.owner != OWN_RESIDENT ||
        e.addr != mig_dst_addr) begin
      errors++;
      $display("FAIL M2 committed entry: %h", e);
    end
    // exactly-once: every diverted token was replayed exactly once
    if ((r_dir[0] - dir0) != (rep_ok_cnt - 0 - rok0)) begin
      errors++;
      $display("FAIL M2 conservation: dir %0d rep_ok %0d",
               r_dir[0] - dir0, rep_ok_cnt);
    end
    if (rep_drop_cnt != rdrop0) begin
      errors++;
      $display("FAIL M2 unexpected drops");
    end
    check_eng(2, r_dir[0] - dir0, 0, 0, 0, "M2");
    for (i = 0; i < 20; i++) begin
      key = gen_key(400 + i);
      send_pkt(key, 2200 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("M2b");
    check_ctr_all(0, "M2b");
    $display("M2 demote-hot done (div=%0d rep=%0d)", r_dir[0] - dir0, rep_ok_cnt);

    // ================= M3: promote a hot block L1 -> L0 ===============
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 8;
    D.width = WID_8; D.seed0 = 32'h55555555; D.seed1 = 32'h66666666;
    D.seed2 = 32'h77777777; D.seed3 = 32'h88888888; D.gen = 8'd3;
    setup_query(1, D, 2, TIER_L1);
    for (i = 0; i < 25; i++) begin
      key = gen_key(500 + (i % 20));
      send_pkt(key, 3000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    // pre-check before M3: q1 L1 src must already match between DUT/model
    check_all("M3pre");
    dir0   = r_dir[1];
    rdrop0 = rep_drop_cnt;
    rok0   = rep_ok_cnt;
    do_mig(MIG_PROMOTE, 1, 0, l0_alloc * 64);
    l0_alloc++;
    for (i = 0; i < 12; i++) begin
      key = gen_key(600 + i);
      send_pkt(key, 3100 + i);
      sync_sdir(1, 0);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    wait_done("M3");
    sync_sdir(1, 0);
    e = sdir_get(1, 0);
    if (!e.valid || e.tier != TIER_L0 || e.owner != OWN_RESIDENT ||
        e.addr != mig_dst_addr) begin
      errors++;
      $display("FAIL M3 committed entry: %h", e);
    end
    if ((r_dir[1] - dir0) != (rep_ok_cnt - rok0)) begin
      errors++;
      $display("FAIL M3 conservation: dir %0d rep_ok %0d",
               r_dir[1] - dir0, rep_ok_cnt - rok0);
    end
    for (i = 0; i < 15; i++) begin
      key = gen_key(700 + i);
      send_pkt(key, 3200 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("M3b");
    check_ctr_all(1, "M3b");
    $display("M3 promote-hot done (div=%0d)", r_dir[1] - dir0);

    // ================= M4: epoch guard drops stale tokens =============
    dir0   = r_dir[0];
    rdrop0 = rep_drop_cnt;
    do_mig(MIG_DEMOTE, 0, 0, l1_alloc * 256);
    l1_alloc++;
    // wait until the freeze is visible, then every packet must divert
    begin
      int t;
      t = 0;
      while (sdir_get(0, 0).owner == OWN_RESIDENT && t < 2000) begin
        sync_sdir(0, 0);
        t++;
      end
      if (sdir_get(0, 0).owner == OWN_RESIDENT) begin
        errors++;
        $display("FAIL M4: freeze never happened");
      end
    end
    // reconfigure the query NOW: every token diverted below is stale from
    // the moment it enters the FIFO.  (Bumping the gen after the packets
    // raced the replay phase: with the HASH_WAIT row latency the copy can
    // finish and start replaying before all 6 packets have diverted, and
    // the early tokens then replay OK instead of dropping.)
    qd[0].gen = qd[0].gen + 8'd1;
    for (i = 0; i < 6; i++) begin
      key = gen_key(800 + i);
      send_pkt(key, 4000 + i);
      sync_sdir(0, 0);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    if (r_dir[0] - dir0 != 6) begin
      errors++;
      $display("FAIL M4: expected 6 diverts, got %0d", r_dir[0] - dir0);
    end
    wait_done("M4");
    sync_sdir(0, 0);
    e = sdir_get(0, 0);
    if (!e.valid || e.tier != TIER_L1 || e.owner != OWN_RESIDENT) begin
      errors++;
      $display("FAIL M4 committed entry: %h", e);
    end
    // replay drain takes HASH_WAIT extra cycles per token now; poll the
    // drop counter instead of sampling it once right after busy falls
    begin
      int t;
      t = 0;
      while ((rep_drop_cnt - rdrop0) != 6 && t < 20000) begin
        @(negedge clk); t++;
      end
    end
    if ((rep_drop_cnt - rdrop0) != 6) begin
      errors++;
      $display("FAIL M4: expected 6 drops, got %0d", rep_drop_cnt - rdrop0);
    end
    check_all("M4");       // destination holds the pure copy, no token writes
    check_ctr_all(0, "M4");
    $display("M4 stale-epoch done (drops=%0d)", rep_drop_cnt - rdrop0);

    // ================= M5: reject a non-RESIDENT block ================
    // pretend block 1 is frozen by "someone else"; the command must be
    // rejected without touching anything
    e = sdir_get(0, 1);
    e.owner = OWN_MIGRATING;
    e.mslot = 4'd3;
    dir_set(0, 1, e);
    begin
      int m0;
      m0 = mig_cnt;
      do_mig(MIG_DEMOTE, 0, 1, 32'hDEAD00);
      mig_mirror_done = 1'b1;   // rejected command: cancel the lazy mirror
      wait_done("M5");
      if (mig_cnt != m0) begin
        errors++;
        $display("FAIL M5: rejected command still migrated");
      end
      check_eng(4, rep_ok_cnt, rep_drop_cnt, 1, 0, "M5");
      // restore the entry (undo the fake freeze)
      e = sdir_get(0, 1);
      e.owner = OWN_RESIDENT;
      e.mslot = 4'd0;
      dir_set(0, 1, e);
    end
    $display("M5 reject done");

    // ================= M6: back-to-back migrations ====================
    do_mig(MIG_PROMOTE, 1, 1, l0_alloc * 64);
    l0_alloc++;
    wait_done("M6a");
    sync_sdir(1, 1);
    // q0 block 0 was demoted to L1 in M4 -- promote it back
    do_mig(MIG_PROMOTE, 0, 0, l0_alloc * 64);
    l0_alloc++;
    wait_done("M6b");
    sync_sdir(0, 0);
    check_eng(6, rep_ok_cnt, rep_drop_cnt, 1, 0, "M6");
    for (i = 0; i < 20; i++) begin
      key = gen_key(900 + i);
      send_pkt(key, 5000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("M6c");
    check_ctr_all(0, "M6c");
    check_ctr_all(1, "M6c");
    $display("M6 back-to-back done");

    // ================= summary ========================================
    repeat (4) @(negedge clk);
    if (errors == 0)
      $display("[tb_mig_engine] ALL TESTS PASSED");
    else
      $display("[tb_mig_engine] %0d ERRORS", errors);
    $finish;
  end

  // safety net
  initial begin
    #20_000_000;
    $display("[tb_mig_engine] TIMEOUT");
    $finish;
  end

endmodule
