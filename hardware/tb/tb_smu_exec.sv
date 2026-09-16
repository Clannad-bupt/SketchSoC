// SMU execution engine regression.
//
// Drives packets / replay tokens through smu_exec against the real
// l0_store, state_directory and heat_table, plus a behavioural L1
// (AXI4-lite subset) memory model.  A software reference model mirrors
// the engine's exact update schedule per query kind:
//
//   CMS/HH : d lookups (diversion aborts the whole op), d reads,
//            v = min+1, d writes of max(cell, v); HH candidate table
//   HLL    : bucket = hash top bits, cell = max(cell, CLZ+1)
//   ENT    : cell = cell + 1 (saturation counted)
//   BLM    : k x (word |= 1 << bit), immediate per hash
//
// Everything the engine may touch -- the whole of L0, the whole of L1,
// per-query counters, HH candidates, heat -- is compared against the
// model after each test, so a stray write anywhere fails the test.

`timescale 1ns/1ps

module tb_smu_exec;

  import sketchsoc_pkg::*;

  localparam int unsigned QBLK_AW  = 4;    // 16 blocks per query
  localparam int unsigned L0_AW    = 11;   // 2048 words = 32 blocks
  localparam int unsigned L0_WORDS = 1 << L0_AW;
  localparam int unsigned L1_LINES = 2048;
  localparam int unsigned DIR_IDXW = QID_W + QBLK_AW;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  int errors = 0;

  // ------------------------------------------------------------------
  // DUT + memories
  // ------------------------------------------------------------------
  qdesc_t qd [QNUM];

  logic        wi_val, wi_rdy, wo_val, wi_is_token, wo_is_token;
  pkt_desc_t   wi_pkt;
  token_t      wi_token;
  logic        wi_gen_ok;
  logic [2:0]  wi_ovr_qid;
  logic [15:0] wi_ovr_blk;
  logic [31:0] wi_ovr_addr;
  logic [1:0]  wi_ovr_tier;
  logic [3:0]  wi_slot, wo_slot;

  logic                dr_val, dr_rdy, dr_dv;
  logic [DIR_IDXW-1:0] dr_idx;
  dirent_t             dr_ent;

  logic             l0r_val, l0r_rdy, l0r_dv;
  logic [L0_AW-1:0] l0r_addr;
  logic [31:0]      l0r_data;
  logic             l0w_val, l0w_rdy;
  logic [L0_AW-1:0] l0w_addr;
  logic [31:0]      l0w_data;
  logic [3:0]       l0w_strb;

  logic                ht_val, ht_rdy;
  logic [DIR_IDXW-1:0] ht_idx;

  logic [MIG_SLOTS-1:0] div_full;
  logic                 div_val, div_rdy;
  logic [3:0]           div_slot;
  token_t               div_tok;

  logic        l1_arvalid, l1_arready, l1_rvalid, l1_rready;
  logic [31:0] l1_araddr;
  logic [63:0] l1_rdata;
  logic        l1_awvalid, l1_awready, l1_bvalid, l1_bready;
  logic [31:0] l1_awaddr;
  logic [63:0] l1_awdata;
  logic [7:0]  l1_awstrb;

  logic [4:0]  acc_sel;
  logic [31:0] acc_rd;
  logic [2:0]  cand_q;
  logic [3:0]  cand_i;
  logic [47:0] cand_rd;

  // directory requestors 1/2 and write ports, L0 requestor 1 / write 1,
  // heat scan port: driven only by this TB
  logic                d_r1_val, d_r1_rdy, d_r1_dv, d_r2_val, d_r2_rdy, d_r2_dv;
  logic [DIR_IDXW-1:0] d_r1_idx, d_r2_idx, d_w0_idx, d_w1_idx;
  dirent_t             d_r1_ent, d_r2_ent, d_w0_ent, d_w1_ent;
  logic                d_w0_val, d_w0_rdy, d_w1_val, d_w1_rdy;

  logic             l0_r1_val, l0_r1_rdy, l0_r1_dv, l0_w1_val, l0_w1_rdy;
  logic [L0_AW-1:0] l0_r1_addr, l0_w1_addr;
  logic [31:0]      l0_r1_data, l0_w1_data;
  logic [3:0]       l0_w1_strb;

  logic                h_scn_val, h_scn_rdy, h_scn_dv, h_scn_decay;
  logic [DIR_IDXW-1:0] h_scn_idx;
  logic [15:0]         h_scn_heat;

  smu_exec #(.QBLK_AW(QBLK_AW), .L0_AW(L0_AW)) dut (
    .clk(clk), .rst_n(rst_n),
    .wi_val(wi_val), .wi_rdy(wi_rdy), .wi_pkt(wi_pkt),
    .wi_is_token(wi_is_token), .wi_token(wi_token), .wi_gen_ok(wi_gen_ok),
    .wi_ovr_qid(wi_ovr_qid), .wi_ovr_blk(wi_ovr_blk),
    .wi_ovr_addr(wi_ovr_addr), .wi_ovr_tier(wi_ovr_tier),
    .wi_slot(wi_slot),
    .wo_val(wo_val), .wo_slot(wo_slot), .wo_is_token(wo_is_token),
    .qd(qd),
    .dr_val(dr_val), .dr_rdy(dr_rdy), .dr_idx(dr_idx),
    .dr_ent(dr_ent), .dr_dv(dr_dv),
    .l0r_val(l0r_val), .l0r_rdy(l0r_rdy), .l0r_addr(l0r_addr),
    .l0r_data(l0r_data), .l0r_dv(l0r_dv),
    .l0w_val(l0w_val), .l0w_rdy(l0w_rdy), .l0w_addr(l0w_addr),
    .l0w_data(l0w_data), .l0w_strb(l0w_strb),
    .ht_val(ht_val), .ht_rdy(ht_rdy), .ht_idx(ht_idx),
    .div_full(div_full), .div_val(div_val), .div_slot(div_slot),
    .div_tok(div_tok), .div_rdy(div_rdy),
    .l1_arvalid(l1_arvalid), .l1_arready(l1_arready), .l1_araddr(l1_araddr),
    .l1_rvalid(l1_rvalid), .l1_rready(l1_rready), .l1_rdata(l1_rdata),
    .l1_awvalid(l1_awvalid), .l1_awready(l1_awready), .l1_awaddr(l1_awaddr),
    .l1_awdata(l1_awdata), .l1_awstrb(l1_awstrb),
    .l1_bvalid(l1_bvalid), .l1_bready(l1_bready),
    .acc_sel(acc_sel), .acc_rd(acc_rd),
    .cand_q(cand_q), .cand_i(cand_i), .cand_rd(cand_rd)
  );

  l0_store #(.AW(L0_AW)) u_l0 (
    .clk(clk), .rst_n(rst_n),
    .r0_val(l0r_val), .r0_rdy(l0r_rdy), .r0_addr(l0r_addr),
    .r0_data(l0r_data), .r0_dv(l0r_dv),
    .r1_val(l0_r1_val), .r1_rdy(l0_r1_rdy), .r1_addr(l0_r1_addr),
    .r1_data(l0_r1_data), .r1_dv(l0_r1_dv),
    .w0_val(l0w_val), .w0_rdy(l0w_rdy), .w0_addr(l0w_addr),
    .w0_data(l0w_data), .w0_strb(l0w_strb),
    .w1_val(l0_w1_val), .w1_rdy(l0_w1_rdy), .w1_addr(l0_w1_addr),
    .w1_data(l0_w1_data), .w1_strb(l0_w1_strb)
  );

  state_directory #(.QBLK_AW(QBLK_AW)) u_dir (
    .clk(clk), .rst_n(rst_n),
    .r0_val(dr_val), .r0_rdy(dr_rdy), .r0_idx(dr_idx),
    .r0_ent(dr_ent), .r0_dv(dr_dv),
    .r1_val(d_r1_val), .r1_rdy(d_r1_rdy), .r1_idx(d_r1_idx),
    .r1_ent(d_r1_ent), .r1_dv(d_r1_dv),
    .r2_val(d_r2_val), .r2_rdy(d_r2_rdy), .r2_idx(d_r2_idx),
    .r2_ent(d_r2_ent), .r2_dv(d_r2_dv),
    .r3_val(1'b0), .r3_idx('0),          // control-plane port: unused here
    .w0_val(d_w0_val), .w0_rdy(d_w0_rdy), .w0_idx(d_w0_idx),
    .w0_ent(d_w0_ent),
    .w1_val(d_w1_val), .w1_rdy(d_w1_rdy), .w1_idx(d_w1_idx),
    .w1_ent(d_w1_ent)
  );

  heat_table #(.QBLK_AW(QBLK_AW)) u_heat (
    .clk(clk), .rst_n(rst_n),
    .acc_val(ht_val), .acc_rdy(ht_rdy), .acc_idx(ht_idx),
    .scn_val(h_scn_val), .scn_rdy(h_scn_rdy), .scn_idx(h_scn_idx),
    .scn_decay(h_scn_decay), .scn_heat(h_scn_heat), .scn_dv(h_scn_dv)
  );

  // ------------------------------------------------------------------
  // behavioural L1 (AXI4-lite subset): 64-bit lines, strobed writes,
  // few-cycle read latency, B after AW
  // ------------------------------------------------------------------
  logic [63:0] l1_mem [0:L1_LINES-1];

  initial begin
    for (int i = 0; i < L1_LINES; i++) l1_mem[i] = 64'h0;
  end

  logic [31:0]  ar_addr_q, wr_addr_q;
  logic [63:0]  wr_data_q;
  logic [7:0]   wr_strb_q;
  int unsigned  rd_dly;
  logic         rd_pend, wr_pend;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      l1_rvalid <= 1'b0; rd_pend <= 1'b0; rd_dly <= 0;
      l1_bvalid <= 1'b0; wr_pend <= 1'b0;
    end else begin
      // read channel
      if (l1_arvalid) begin                    // arready = 1
        rd_pend   <= 1'b1;
        rd_dly    <= 3;
        ar_addr_q <= l1_araddr;
      end
      if (rd_pend && (rd_dly == 0)) begin
        if (l1_rvalid && l1_rready) begin
          l1_rvalid <= 1'b0;
          rd_pend   <= 1'b0;
        end else begin
          l1_rvalid <= 1'b1;
          l1_rdata  <= l1_mem[ar_addr_q[31:3] % L1_LINES];
        end
      end else if (rd_pend) begin
        rd_dly <= rd_dly - 1;
      end
      // write channel
      if (l1_awvalid) begin                    // awready = 1
        wr_pend   <= 1'b1;
        wr_addr_q <= l1_awaddr;
        wr_data_q <= l1_awdata;
        wr_strb_q <= l1_awstrb;
      end
      if (wr_pend) begin
        for (int b = 0; b < 8; b++)
          if (wr_strb_q[b])
            l1_mem[wr_addr_q[31:3] % L1_LINES][8*b +: 8] <= wr_data_q[8*b +: 8];
        l1_bvalid <= 1'b1;
        wr_pend   <= 1'b0;
      end
      if (l1_bvalid && l1_bready)
        l1_bvalid <= 1'b0;
    end
  end

  // divert capture (FIFO side always ready in this TB)
  token_t div_cap [0:255];
  int     div_cap_n = 0;
  int     div_cap_slot [0:255];
  always @(posedge clk) begin
    if (div_val && div_rdy) begin
      div_cap[div_cap_n]      = div_tok;
      div_cap_slot[div_cap_n] = div_slot;
      div_cap_n               = div_cap_n + 1;
    end
  end

  assign l1_arready = 1'b1;
  assign l1_awready = 1'b1;
  assign div_rdy    = 1'b1;

  // ------------------------------------------------------------------
  // reference model
  // ------------------------------------------------------------------
  dirent_t     sdir [int];        // shadow directory, key = q*4096+blk
  logic [31:0] exp_l0 [int];      // expected L0 words (default 0)
  logic [63:0] exp_l1 [int];      // expected L1 lines (default 0)
  int          r_att [0:QNUM-1];
  int          r_rej [0:QNUM-1];
  int          r_dir [0:QNUM-1];
  int          r_rep [0:QNUM-1];
  int          r_sat [0:QNUM-1];
  logic        rc_valid [0:QNUM-1][0:MAX_CAND-1];
  logic [15:0] rc_fp    [0:QNUM-1][0:MAX_CAND-1];
  logic [31:0] rc_cnt   [0:QNUM-1][0:MAX_CAND-1];

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

  // cell geometry from cell index (mirrors the DUT address generator)
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

  task automatic ref_hh(input int q, input logic [15:0] fp,
                        input logic [31:0] v);
    int hit, mini;
    logic [31:0] minv;
    hit  = -1;
    mini = 0;
    minv = 32'hFFFFFFFF;
    for (int i = 0; i < MAX_CAND; i++) begin
      if (i < (1 << qd[q].cand_log2)) begin
        if (rc_valid[q][i] && (rc_fp[q][i] == fp) && (hit < 0)) hit = i;
        if (rc_cnt[q][i] < minv) begin
          minv = rc_cnt[q][i];
          mini = i;
        end
      end
    end
    if (hit >= 0) begin
      if (v > rc_cnt[q][hit]) rc_cnt[q][hit] = v;
    end else if (v > minv) begin
      rc_valid[q][mini] = 1'b1;
      rc_fp[q][mini]    = fp;
      rc_cnt[q][mini]   = v;
    end
  endtask

  // mirror of the DUT update for one (query, work item)
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
          // phased: lookup + read all rows (diversion aborts whole op)
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
          if (D.kind == KIND_HH) begin
            h = row_hash(0, fam, D.seed0, D.seed1, D.seed2, D.seed3, key);
            ref_hh(q, h[15:0], v);
          end
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
  // driver / checker tasks
  // ------------------------------------------------------------------
  task automatic dir_set(input int q, input int blk, input dirent_t e);
    begin
      @(negedge clk);
      d_w1_val = 1'b1;
      d_w1_idx = (q << QBLK_AW) + blk;
      d_w1_ent = e;
      while (!d_w1_rdy) @(negedge clk);
      @(negedge clk);
      d_w1_val = 1'b0;
      sdir[q*4096 + blk] = e;
    end
  endtask

  // allocate n contiguous L0 (or L1) blocks, install RESIDENT entries
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

  task automatic disable_query(input int q);
    qdesc_t D;
    begin
      D = qd[q];
      D.enable = 1'b0;
      qd[q] = D;
    end
  endtask

  task automatic send_pkt(input flow_key_t key, input int id);
    begin
      @(negedge clk);
      wi_val        = 1'b1;
      wi_is_token   = 1'b0;
      wi_token      = '0;
      wi_gen_ok     = 1'b1;
      wi_pkt.key    = key;
      wi_pkt.pkt_id = id;
      wi_pkt.tstamp = 0;
      wi_pkt.rsv    = '0;
      wi_slot       = 0;
      while (!wi_rdy) @(negedge clk);
      @(negedge clk);
      wi_val = 1'b0;
      while (!wo_val) @(negedge clk);
    end
  endtask

  task automatic send_token(input flow_key_t key, input int q,
                            input bit gen_ok, input int ovr_blk,
                            input logic [31:0] ovr_addr,
                            input logic [1:0] ovr_tier);
    begin
      @(negedge clk);
      wi_val        = 1'b1;
      wi_is_token   = 1'b1;
      wi_gen_ok     = gen_ok;
      wi_token      = '0;
      wi_token.key  = key;
      wi_token.kind = qd[q].kind;
      wi_token.qid  = q;
      wi_ovr_qid    = q;
      wi_ovr_blk    = ovr_blk;
      wi_ovr_addr   = ovr_addr;
      wi_ovr_tier   = ovr_tier;
      wi_slot       = 4'd1;
      while (!wi_rdy) @(negedge clk);
      @(negedge clk);
      wi_val = 1'b0;
      while (!wo_val) @(negedge clk);
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

  task automatic check_cands(input int q, input string tag);
    logic [47:0] got;
    begin
      for (int i = 0; i < (1 << qd[q].cand_log2); i++) begin
        cand_q = q;
        cand_i = i;
        @(negedge clk);
        got = cand_rd;
        if (rc_valid[q][i]) begin
          if (got !== {rc_fp[q][i], rc_cnt[q][i]}) begin
            errors++;
            $display("FAIL %s cand q%0d i%0d: got %h exp %h", tag, q, i,
                     got, {rc_fp[q][i], rc_cnt[q][i]});
          end
        end else begin
          if (got !== 48'h0) begin
            errors++;
            $display("FAIL %s cand q%0d i%0d: got %h exp 0", tag, q, i, got);
          end
        end
      end
    end
  endtask

  task automatic heat_probe(input int q, input int blk,
                            output logic [15:0] h);
    begin
      @(negedge clk);
      h_scn_val   = 1'b1;
      h_scn_idx   = (q << QBLK_AW) + blk;
      h_scn_decay = 1'b0;
      while (!h_scn_dv) @(negedge clk);
      h = h_scn_heat;
      h_scn_val = 1'b0;
    end
  endtask

  task automatic check_ctr_all(input int q, input string tag);
    check_ctr(q, 0, r_att[q], tag);
    check_ctr(q, 1, r_rej[q], tag);
    check_ctr(q, 2, r_att[q] - r_rej[q] - r_dir[q], tag);  // direct
    check_ctr(q, 3, r_dir[q], tag);
    check_ctr(q, 4, r_rep[q], tag);
    check_ctr(q, 5, r_sat[q], tag);
  endtask

  // second HLL descriptor for the multi-query test
  function automatic qdesc_t D_HLL9();
    qdesc_t t;
    t = '0;
    t.enable = 1'b1; t.kind = KIND_HLL; t.cols_log2 = 8;
    t.width = WID_8; t.gen = 8'd9;
    t.seed0 = 32'h2468ACE0; t.seed1 = 32'h10203040;
    t.seed2 = 32'h50607080; t.seed3 = 32'h90A0B0C0;
    return t;
  endfunction

  // ------------------------------------------------------------------
  // stimulus
  // ------------------------------------------------------------------
  qdesc_t D;
  dirent_t e;
  flow_key_t key;
  logic [15:0] heat, hexp;
  int i;

  initial begin
    // defaults
    wi_val = 0; wi_is_token = 0; wi_pkt = '0; wi_token = '0; wi_gen_ok = 1;
    wi_ovr_qid = 0; wi_ovr_blk = 0; wi_ovr_addr = 0; wi_ovr_tier = 0;
    wi_slot = 0;
    div_full = '0;
    acc_sel = 0; cand_q = 0; cand_i = 0;
    d_r1_val = 0; d_r2_val = 0; d_w0_val = 0; d_w1_val = 0;
    d_r1_idx = 0; d_r2_idx = 0; d_w0_idx = 0; d_w1_idx = 0;
    d_w0_ent = '0; d_w1_ent = '0;
    l0_r1_val = 0; l0_w1_val = 0; l0_r1_addr = 0; l0_w1_addr = 0;
    l0_w1_data = 0; l0_w1_strb = 0;
    h_scn_val = 0; h_scn_decay = 0; h_scn_idx = 0;
    for (int q = 0; q < QNUM; q++) begin
      qd[q] = '0;
      r_att[q] = 0; r_rej[q] = 0; r_dir[q] = 0; r_rep[q] = 0; r_sat[q] = 0;
      for (int c = 0; c < MAX_CAND; c++) begin
        rc_valid[q][c] = 0; rc_fp[q][c] = 0; rc_cnt[q][c] = 0;
      end
    end

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ================= T1: CMS 8-bit, 3 rows ==========================
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 3; D.cols_log2 = 8;
    D.width = WID_8; D.seed0 = 32'h11111111; D.seed1 = 32'h22222222;
    D.seed2 = 32'h33333333; D.seed3 = 32'h44444444; D.gen = 8'd1;
    setup_query(0, D, 3, TIER_L0);
    for (i = 0; i < 60; i++) begin
      key = gen_key(i % 20);
      send_pkt(key, i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T1");
    check_ctr_all(0, "T1");
    $display("T1 cms-8bit done");

    // ================= T1b: CMS 16-bit, 2 rows ========================
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 7;
    D.width = WID_16; D.seed0 = 32'h55555555; D.seed1 = 32'h66666666;
    D.seed2 = 32'h77777777; D.seed3 = 32'h88888888; D.gen = 8'd2;
    setup_query(0, D, 2, TIER_L0);
    for (i = 0; i < 50; i++) begin
      key = gen_key(100 + (i % 30));
      send_pkt(key, 1000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T1b");
    check_ctr_all(0, "T1b");
    $display("T1b cms-16bit done");

    // ================= T2: HH (CMS + candidates) ======================
    D = '0;
    D.enable = 1'b1; D.kind = KIND_HH; D.rows = 3; D.cols_log2 = 8;
    D.width = WID_8; D.cand_log2 = 4; D.gen = 8'd3;
    D.seed0 = 32'h99999999; D.seed1 = 32'hAAAAAAAA;
    D.seed2 = 32'hBBBBBBBB; D.seed3 = 32'hCCCCCCCC;
    setup_query(1, D, 3, TIER_L0);
    disable_query(0);
    key = gen_key(7);                       // heavy key
    for (i = 0; i < 25; i++) begin
      send_pkt(key, 2000 + i);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    for (i = 0; i < 15; i++) begin
      key = gen_key(500 + i);
      send_pkt(key, 2100 + i);
      ref_apply(1, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T2");
    check_ctr_all(1, "T2");
    check_cands(1, "T2");
    disable_query(1);
    $display("T2 hh done");

    // ================= T3: HLL ========================================
    D = '0;
    D.enable = 1'b1; D.kind = KIND_HLL; D.cols_log2 = 8;
    D.width = WID_8; D.gen = 8'd4;
    D.seed0 = 32'hDDDDDDDD; D.seed1 = 32'hEEEEEEEE;
    D.seed2 = 32'hFFFFFFFF; D.seed3 = 32'h01234567;
    setup_query(2, D, 1, TIER_L0);
    for (i = 0; i < 40; i++) begin
      key = gen_key(i * 11);
      send_pkt(key, 3000 + i);
      ref_apply(2, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T3");
    check_ctr_all(2, "T3");
    $display("T3 hll done");

    // ================= T4: ENT in L1, saturation + heat ===============
    D = '0;
    D.enable = 1'b1; D.kind = KIND_ENT; D.cols_log2 = 4;
    D.width = WID_8; D.gen = 8'd5;
    D.seed0 = 32'h89ABCDEF; D.seed1 = 32'h0F0F0F0F;
    D.seed2 = 32'hF0F0F0F0; D.seed3 = 32'h13579BDF;
    setup_query(3, D, 1, TIER_L1);
    disable_query(2);
    for (i = 0; i < 20; i++) begin
      key = gen_key(900 + i);
      send_pkt(key, 4000 + i);
      ref_apply(3, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    key = gen_key(77);
    for (i = 0; i < 300; i++) begin
      send_pkt(key, 4100 + i);
      ref_apply(3, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T4");
    check_ctr_all(3, "T4");
    // heat: 320 applied ops on q3 block 0
    hexp = 16'd0;
    for (i = 0; i < 320; i++) hexp = hexp + 16'd512 - (hexp >> 3);
    heat_probe(3, 0, heat);
    if (heat !== hexp) begin
      errors++;
      $display("FAIL T4 heat: got %0d exp %0d", heat, hexp);
    end
    disable_query(3);
    $display("T4 ent-l1 done (sat=%0d)", r_sat[3]);

    // ================= T5: BLM ========================================
    D = '0;
    D.enable = 1'b1; D.kind = KIND_BLM; D.cols_log2 = 6; D.blm_k = 3;
    D.width = WID_32; D.gen = 8'd6;
    D.seed0 = 32'h2468ACE0; D.seed1 = 32'h13572468;
    D.seed2 = 32'h00112233; D.seed3 = 32'hCAFEF00D;
    setup_query(0, D, 1, TIER_L0);
    for (i = 0; i < 30; i++) begin
      key = gen_key(2000 + (i % 18));
      send_pkt(key, 5000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T5");
    check_ctr_all(0, "T5");
    $display("T5 blm done");

    // ================= T6: diversion + FIFO-full ======================
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 3; D.cols_log2 = 8;
    D.width = WID_8; D.gen = 8'd7;
    D.seed0 = 32'hA1B2C3D4; D.seed1 = 32'hE5F60718;
    D.seed2 = 32'h192A3B4C; D.seed3 = 32'h5D6E7F80;
    setup_query(0, D, 3, TIER_L0);
    // freeze block 1 (row 1 of this config maps to block 1) in slot 2
    e        = '0;
    e.valid  = 1'b1;
    e.tier   = TIER_L0;
    e.owner  = OWN_MIGRATING;
    e.gen    = 8'd7;
    e.mslot  = 4'd2;
    e.addr   = sdir_get(0, 1).addr;
    dir_set(0, 1, e);
    for (i = 0; i < 5; i++) begin
      key = gen_key(3000 + i);
      send_pkt(key, 6000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    if (div_cap_n != 5) begin
      errors++;
      $display("FAIL T6 divert count: got %0d exp 5", div_cap_n);
    end
    for (i = 0; i < 5; i++) begin
      if (div_cap_slot[i] != 2) begin
        errors++;
        $display("FAIL T6 divert slot: got %0d exp 2", div_cap_slot[i]);
      end
    end
    check_all("T6a");
    check_ctr_all(0, "T6a");
    // now backpressure the FIFO: same frozen block, FIFO full -> reject
    div_full = 8'b0000_0100;
    for (i = 0; i < 5; i++) begin
      key = gen_key(3100 + i);
      send_pkt(key, 6100 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    if (div_cap_n != 5) begin
      errors++;
      $display("FAIL T6b: tokens enqueued while FIFO full (%0d)", div_cap_n);
    end
    div_full = '0;
    check_all("T6b");
    check_ctr_all(0, "T6b");
    $display("T6 divert done (dir=%0d rej=%0d)", r_dir[0], r_rej[0]);

    // ================= T7: replay override ============================
    // block 1 stays frozen (REPLAYING); replay tokens redirect its rows
    // to a fresh unpublished destination block.
    e       = sdir_get(0, 1);
    e.owner = OWN_REPLAYING;
    e.mslot = 4'd2;
    dir_set(0, 1, e);
    for (i = 0; i < 8; i++) begin
      key = gen_key(4000 + i);
      send_token(key, 0, 1'b1, 1, l0_alloc * 64, TIER_L0);
      ref_apply(0, key, 1'b1, 0, 1, l0_alloc * 64, TIER_L0);
    end
    l0_alloc++;                       // the destination block is now used
    check_all("T7");
    check_ctr_all(0, "T7");
    // unfreeze
    e       = sdir_get(0, 1);
    e.owner = OWN_RESIDENT;
    e.mslot = 0;
    dir_set(0, 1, e);
    $display("T7 replay done (rep=%0d)", r_rep[0]);

    // ================= T8: stale-epoch token ==========================
    key = gen_key(5000);
    send_token(key, 0, 1'b0, 0, 32'h0, TIER_L0);
    // no ref_apply: nothing may happen
    check_all("T8");
    $display("T8 stale token done");

    // ================= T9: multi-query dispatch =======================
    setup_query(2, D_HLL9(), 1, TIER_L0);       // q2: HLL again (fresh)
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 8;
    D.width = WID_8; D.gen = 8'd9;
    D.seed0 = 32'h0BADF00D; D.seed1 = 32'hD15EA5E;
    D.seed2 = 32'hFEEDFACE; D.seed3 = 32'hDEADBEEF;
    setup_query(0, D, 2, TIER_L0);
    for (i = 0; i < 25; i++) begin
      key = gen_key(6000 + i);
      send_pkt(key, 7000 + i);
      ref_apply(0, key, 1'b0, 0, 0, 32'h0, TIER_L0);
      ref_apply(2, key, 1'b0, 0, 0, 32'h0, TIER_L0);
    end
    check_all("T9");
    check_ctr_all(0, "T9");
    check_ctr_all(2, "T9");
    $display("T9 multi-query done");

    // ================= summary ========================================
    repeat (4) @(negedge clk);
    if (errors == 0)
      $display("[tb_smu_exec] ALL TESTS PASSED");
    else
      $display("[tb_smu_exec] %0d ERRORS", errors);
    $finish;
  end

  // safety net
  initial begin
    #5_000_000;
    $display("[tb_smu_exec] TIMEOUT");
    $finish;
  end

endmodule
