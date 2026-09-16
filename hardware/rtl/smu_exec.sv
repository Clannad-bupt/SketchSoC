// SMU execution engine (paper Sec. 3.4.1, Fig. 5, Table 2).
//
// One logical 256-bit lane: given a packet descriptor (or a replay token,
// which re-executes the same schedule), it runs the update programme of
// every enabled query:
//
//   CMS/HH : d x [HASH -> directory lookup]; on the first frozen block the
//            whole operation diverts as ONE 128-bit logical token to that
//            block's migration-slot FIFO.  Otherwise d x [read cell],
//            v = min(cells)+1 (conservative Count-Min), d x [write
//            max(cell, v)] -- the paper's MIN/ADD/MAX bank-parallel
//            primitives.  HH then updates a bounded candidate table with
//            a statically scheduled compare + select.
//   HLL    : HASH -> bucket = top bits, rank = CLZ(low bits)+1,
//            cell = MAX(cell, rank).
//   ENT    : HASH -> bucket, cell = cell + 1 (saturation counted).
//   BLM    : k x [HASH -> read word, OR bit, write word].
//
// Tier routing per block comes from the directory: RESIDENT blocks are
// updated in place (L0 word RMW, or an L1 single-beat read then a
// strobe-gated write); a block owned by MIGRATING/REPLAYING diverts the
// operation.  If the target FIFO is full the operation is REJECTED and
// counted (Eq. 1) -- never silently dropped, never head-of-line blocking.
//
// Replay work items carry a block override {qid, block, dest addr, tier}:
// accesses to the migrating block are redirected to the unpublished
// destination; accesses to other frozen blocks re-divert into those
// blocks' FIFOs (a later replay pass picks them up).
//
// Heat: one sampled block per applied operation (row rot_ctr mod rows)
// keeps per-block EWMA comparable across rows at one heat op per packet.

`timescale 1ns/1ps

module smu_exec import sketchsoc_pkg::*; #(
  parameter int unsigned QBLK_AW = 12,
  parameter int unsigned L0_AW   = 12
)(
  input  logic clk,
  input  logic rst_n,

  // ---- work input (packet or replay token; top muxes priority) --------
  input  logic        wi_val,
  output logic        wi_rdy,
  input  pkt_desc_t   wi_pkt,
  input  logic        wi_is_token,
  input  token_t      wi_token,
  input  logic        wi_gen_ok,        // engine pre-checked epoch
  input  logic [2:0]  wi_ovr_qid,
  input  logic [15:0] wi_ovr_blk,
  input  logic [31:0] wi_ovr_addr,
  input  logic [1:0]  wi_ovr_tier,
  input  logic [3:0]  wi_slot,
  output logic        wo_val,           // work completed (in order)
  output logic [3:0]  wo_slot,
  output logic        wo_is_token,
  output logic        idle,             // no work item in flight

  // ---- query descriptors (dynamic configuration state) ----------------
  input  qdesc_t      qd [QNUM],

  // ---- directory read (requestor 0) ------------------------------------
  output logic                     dr_val,
  input  logic                     dr_rdy,
  output logic [QID_W+QBLK_AW-1:0] dr_idx,
  input  dirent_t                  dr_ent,
  input  logic                     dr_dv,

  // ---- L0 store: read requestor 0 + write requestor 0 ------------------
  output logic             l0r_val,
  input  logic             l0r_rdy,
  output logic [L0_AW-1:0] l0r_addr,
  input  logic [31:0]      l0r_data,
  input  logic             l0r_dv,
  output logic             l0w_val,
  input  logic             l0w_rdy,
  output logic [L0_AW-1:0] l0w_addr,
  output logic [31:0]      l0w_data,
  output logic [3:0]       l0w_strb,

  // ---- heat access port ------------------------------------------------
  output logic                     ht_val,
  input  logic                     ht_rdy,
  output logic [QID_W+QBLK_AW-1:0] ht_idx,

  // ---- divert to migration-slot FIFO -----------------------------------
  input  logic [MIG_SLOTS-1:0] div_full,
  output logic                 div_val,
  output logic [3:0]           div_slot,
  output token_t               div_tok,
  input  logic                 div_rdy,

  // ---- L1 memory command interface (AXI4-lite subset, 64-bit) ----------
  output logic        l1_arvalid,
  input  logic        l1_arready,
  output logic [31:0] l1_araddr,
  input  logic        l1_rvalid,
  output logic        l1_rready,
  input  logic [63:0] l1_rdata,
  output logic        l1_awvalid,
  input  logic        l1_awready,
  output logic [31:0] l1_awaddr,
  output logic [63:0] l1_awdata,
  output logic [7:0]  l1_awstrb,
  input  logic        l1_bvalid,
  output logic        l1_bready,

  // ---- accounting read (control plane) ---------------------------------
  input  logic [4:0]  acc_sel,
  output logic [31:0] acc_rd,

  // ---- HH candidate readout ---------------------------------------------
  input  logic [2:0]  cand_q,
  input  logic [3:0]  cand_i,
  output logic [47:0] cand_rd
);

  // ------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------
  typedef enum logic [4:0] {
    ST_IDLE, ST_QSEL, ST_HASH, ST_LU, ST_LUW, ST_LU_ADV, ST_DIVERT,
    ST_RD, ST_RDW, ST_L1RD, ST_RD_ADV, ST_COMPUTE, ST_UPD,
    ST_WR, ST_WR_ADV, ST_L1WR, ST_L1WR_W, ST_HEAT, ST_HH, ST_NEXTQ, ST_DONE
  } state_e;

  state_e st;

  // quiescence indicator for the migration engine: the engine must observe
  // idle before a freeze/commit directory CAS, so no in-flight operation can
  // hold a stale (pre-CAS) view of the entry it is about to mutate.
  assign idle = (st == ST_IDLE);

  // work context
  flow_key_t   w_key;
  logic        w_is_token;
  logic        w_gen_ok;
  logic [2:0]  w_tok_qid;
  logic [2:0]  w_ovr_qid;
  logic [15:0] w_ovr_blk;
  logic [31:0] w_ovr_addr;
  logic [1:0]  w_ovr_tier;
  logic [3:0]  w_slot;
  logic [2:0]  w_q;
  logic [3:0]  w_row;
  logic [3:0]  w_rows;
  logic        w_is_cms;      // phased multi-row op (CMS/HH)

  // per-row access info
  logic [1:0]  r_tier   [0:MAX_ROWS-1];
  logic [31:0] r_addr   [0:MAX_ROWS-1];   // L0: word addr | L1: byte addr
  logic [5:0]  r_wib    [0:MAX_ROWS-1];   // word index within block
  logic [3:0]  r_strb   [0:MAX_ROWS-1];
  logic [7:0]  r_bshift [0:MAX_ROWS-1];   // byte shift in word (bit for BLM)
  logic [31:0] r_val    [0:MAX_ROWS-1];   // cell value read
  logic [15:0] r_blk    [0:MAX_ROWS-1];   // logical block (heat sampling)

  // registered hash results
  logic [31:0] hash_reg;
  logic [7:0]  rank_reg;                  // HLL rank
  logic [15:0] fp_reg;                    // HH fingerprint (row 0 hash)
  logic [4:0]  bit_reg;                   // BLM bit index within word
  logic [3:0]  hash_cnt;                  // ST_HASH settle counter

  // ST_HASH settle length: must match the multicycle constraint in
  // scripts/run_synth.tcl (set_multicycle_path HASH_WAIT -to <hash regs>)
  localparam int unsigned HASH_WAIT = 8;

  // diversion target
  logic [3:0]  r_fslot;

  // update value and heat sampling
  logic [31:0] upd_val;
  logic [31:0] min_reg;   // ST_COMPUTE registers the min-tree; ST_UPD adds/sat
  logic [1:0]  rot_ctr;

  // HH candidate table (bounded, always on chip)
  logic        c_valid [0:QNUM-1][0:MAX_CAND-1];
  logic [15:0] c_fp    [0:QNUM-1][0:MAX_CAND-1];
  logic [31:0] c_cnt   [0:QNUM-1][0:MAX_CAND-1];

  // accounting counters: 0 attempted 1 rejected 2 direct 3 diverted
  //                       4 replay_done 5 saturation
  logic [31:0] ctr [0:QNUM-1][0:5];

  always_comb begin
    acc_rd = 32'h0;
    if ((acc_sel[4:3] < QNUM) && (acc_sel[2:0] <= 3'd5))
      acc_rd = ctr[acc_sel[4:3]][acc_sel[2:0]];
  end

  always_comb begin
    if (cand_q < QNUM)
      cand_rd = {c_fp[cand_q][cand_i], c_cnt[cand_q][cand_i]};
    else
      cand_rd = 48'h0;
  end

  // ------------------------------------------------------------------
  // address generation (combinational, from the current key/row/query)
  // ------------------------------------------------------------------
  logic [31:0] cur_hash;
  logic [31:0] cell_lin;      // cell index (word index for BLM)
  logic [31:0] word_lin;      // word index within the query's state
  logic [1:0]  byte_off;
  logic [15:0] cur_blk;
  logic [5:0]  cur_wib;
  logic [7:0]  cur_bshift;
  logic [3:0]  cur_strb;
  logic [31:0] cur_bitidx;
  logic [4:0]  bit_reg_c;     // BLM bit within word (combinational)
  logic [31:0] cols;

  qdesc_t d;
  assign d = qd[w_q[1:0]];
  // multi-cycle hash cone: the row hash is a deep combinational network
  // (CRC32 unrolls to ~128 XOR levels; murmur/xx chain serial fmix
  // multiplies).  It settles while ST_HASH waits HASH_WAIT cycles and is
  // captured into hash_reg/rank_reg/fp_reg/bit_reg at the final edge; those
  // four registers carry a set_multicycle_path of HASH_WAIT (scripts/
  // run_synth.tcl).  Everything downstream addresses from the REGISTERED
  // hash, so the rest of the engine stays single-cycle.
  assign cur_hash = row_hash(w_row, d.kind[1:0], d.seed0, d.seed1,
                             d.seed2, d.seed3, w_key);

  // BLM bit position within the word -- captured with hash_reg in ST_HASH,
  // so it must come from the combinational cone (same multicycle group)
  logic [31:0] blm_idx_c;
  assign blm_idx_c = cur_hash & ((32'd1 << (d.cols_log2 + 5)) - 32'd1);
  assign bit_reg_c = blm_idx_c[4:0];

  always_comb begin
    // top bits of the REGISTERED hash select the column.  The row base is
    // w_row * cols, but cols = 2^cols_log2, so this is a SHIFT -- writing it
    // as a multiply mapped to a DSP48 chain and was the WNS -1.3ns critical
    // path (w_q -> descriptor mux -> DSP -> r_wib).
    cell_lin   = (d.kind == KIND_CMS || d.kind == KIND_HH)
                 ? (({28'd0, w_row} << d.cols_log2) +
                    (hash_reg >> (32 - d.cols_log2)))
                 :  (hash_reg >> (32 - d.cols_log2));
    cur_bitidx = hash_reg & ((32'd1 << (d.cols_log2 + 5)) - 32'd1);
    word_lin   = cell_lin;
    byte_off   = 2'b00;
    cur_wib    = 6'd0;
    cur_blk    = 16'd0;
    cur_bshift = 8'd0;
    cur_strb   = 4'd0;
    case (d.kind)
      KIND_BLM: begin
        // bit array of cols 32-bit words
        word_lin   = cur_bitidx >> 5;
        cur_wib    = word_lin[5:0];
        cur_blk    = word_lin[21:6];
        cur_bshift = {3'b000, cur_bitidx[4:0]};   // bit position
        cur_strb   = 4'b1111;
      end
      default: begin
        case (d.width)
          WID_8: begin
            word_lin = cell_lin >> 2;
            byte_off = cell_lin[1:0];
          end
          WID_16: begin
            word_lin = cell_lin >> 1;
            byte_off = {cell_lin[0], 1'b0};
          end
          default: begin
            word_lin = cell_lin;
            byte_off = 2'b00;
          end
        endcase
        cur_wib    = word_lin[5:0];
        cur_blk    = word_lin[21:6];
        cur_bshift = {byte_off, 3'b000};
        cur_strb   = cell_strb(d.width, byte_off);
      end
    endcase
  end

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------
  function automatic logic [31:0] cell_ext(input logic [31:0] w,
                                           input logic [1:0] wd,
                                           input logic [7:0] bsh);
    case (wd)
      WID_8:  return {24'h0, w >> bsh};
      WID_16: return {16'h0, w >> bsh};
      default: return w;
    endcase
  endfunction

  function automatic logic [31:0] wmax(input logic [1:0] wd);
    case (wd)
      WID_8:  return 32'h000000FF;
      WID_16: return 32'h0000FFFF;
      default: return 32'hFFFFFFFF;
    endcase
  endfunction

  // min over the d row values (CMS/HH conservative update).
  // Balanced 3-level tree (not a serial chain): the serial version alone
  // was ~30 CARRY8 levels and, together with the +1/saturate logic, formed
  // the post-hash-fix critical path (WNS -4.4ns @220MHz).  Inactive rows
  // are masked to all-ones so they lose every compare.
  logic [31:0] min_c;
  always_comb begin
    logic [31:0] v [MAX_ROWS];
    logic [31:0] a0, a1, a2, a3, b0, b1;
    for (int rr = 0; rr < MAX_ROWS; rr++)
      v[rr] = (rr < w_rows) ? r_val[rr] : 32'hFFFFFFFF;
    a0 = (v[0] < v[1]) ? v[0] : v[1];
    a1 = (v[2] < v[3]) ? v[2] : v[3];
    a2 = (v[4] < v[5]) ? v[4] : v[5];
    a3 = (v[6] < v[7]) ? v[6] : v[7];
    b0 = (a0 < a1) ? a0 : a1;
    b1 = (a2 < a3) ? a2 : a3;
    min_c = (b0 < b1) ? b0 : b1;
  end

  // heat sample row: rot_ctr (2 bit) mod rows -- explicit, division-free
  logic [3:0] rot_mod;
  always_comb begin
    case (w_rows)
      4'd1:    rot_mod = 4'd0;
      4'd2:    rot_mod = {3'b0, rot_ctr[0]};
      4'd3:    rot_mod = (rot_ctr == 2'd3) ? 4'd0 : {2'b0, rot_ctr};
      default: rot_mod = {2'b0, rot_ctr};      // rows >= 4
    endcase
  end

  logic [15:0] heat_blk;
  always_comb begin
    if (w_is_cms)
      heat_blk = r_blk[rot_mod];
    else
      heat_blk = r_blk[0];
  end

  // HH candidate search (statically scheduled comparator network)
  logic        hh_hit;
  logic [3:0]  hh_hit_i;
  logic [3:0]  hh_min_i;
  logic [31:0] hh_min_v;
  always_comb begin
    hh_hit   = 1'b0;
    hh_hit_i = 4'd0;
    hh_min_v = 32'hFFFFFFFF;
    hh_min_i = 4'd0;
    for (int i = 0; i < MAX_CAND; i++) begin
      if ((i < (16'd1 << d.cand_log2)) && c_valid[w_q[1:0]][i] &&
          (c_fp[w_q[1:0]][i] == fp_reg) && !hh_hit) begin
        hh_hit   = 1'b1;
        hh_hit_i = i[3:0];
      end
      if (i < (16'd1 << d.cand_log2)) begin
        if (c_cnt[w_q[1:0]][i] < hh_min_v) begin
          hh_min_v = c_cnt[w_q[1:0]][i];
          hh_min_i = i[3:0];
        end
      end
    end
  end

  // per-row write value for the phased CMS/HH loop: max(cell, v)
  logic [31:0] wr_row_val;
  assign wr_row_val = (r_val[w_row] < upd_val) ? upd_val : r_val[w_row];

  // physical addresses of the current row's cell:
  //   L0 : directory addr is a word address, wib is a word offset
  //   L1 : directory addr is a byte address, wib scales by 4
  logic [31:0] l0_word_addr;
  logic [31:0] l1_byte_addr;
  assign l0_word_addr = r_addr[w_row] + {26'h0, r_wib[w_row]};
  assign l1_byte_addr = r_addr[w_row] + ({26'h0, r_wib[w_row]} << 2);

  // 32-bit word to write (CMS/HH: shifted cell; BLM: whole word; else shifted)
  logic [31:0] wr_word;
  always_comb begin
    if (d.kind == KIND_CMS || d.kind == KIND_HH)
      wr_word = wr_row_val << r_bshift[w_row];
    else if (d.kind == KIND_BLM)
      wr_word = upd_val;
    else
      wr_word = upd_val << r_bshift[w_row];
  end

  // L1 64-bit lane select (block bases are 8-byte aligned)
  logic lane;
  assign lane = l1_byte_addr[2];

  // ------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st         <= ST_IDLE;
      dr_val     <= 1'b0;  l0r_val <= 1'b0;  l0w_val <= 1'b0;
      ht_val     <= 1'b0;  div_val <= 1'b0;  wo_val  <= 1'b0;
      l1_arvalid <= 1'b0;  l1_awvalid <= 1'b0;
      l1_rready  <= 1'b0;  l1_bready  <= 1'b0;
      wi_rdy     <= 1'b0;
      rot_ctr    <= 2'd0;
      hash_cnt   <= 4'd0;
      upd_val    <= 32'h0;
      min_reg    <= 32'h0;
      for (int q = 0; q < QNUM; q++)
        for (int c = 0; c < MAX_CAND; c++) begin
          c_valid[q][c] <= 1'b0;
          c_fp[q][c]    <= 16'h0;
          c_cnt[q][c]   <= 32'h0;
        end
      for (int q = 0; q < QNUM; q++)
        for (int c = 0; c < 6; c++)
          ctr[q][c] <= 32'h0;
    end else begin
      // single-cycle pulse defaults (overridden where held)
      dr_val  <= 1'b0;
      l0r_val <= 1'b0;
      l0w_val <= 1'b0;
      ht_val  <= 1'b0;
      wo_val  <= 1'b0;

      case (st)
        // ------------------------------------------------------------
        ST_IDLE: begin
          wi_rdy <= 1'b1;
          if (wi_val && wi_rdy) begin
            w_key      <= wi_is_token ? wi_token.key : wi_pkt.key;
            w_is_token <= wi_is_token;
            w_gen_ok   <= wi_gen_ok;
            w_tok_qid  <= wi_token.qid;
            w_ovr_qid  <= wi_ovr_qid;
            w_ovr_blk  <= wi_ovr_blk;
            w_ovr_addr <= wi_ovr_addr;
            w_ovr_tier <= wi_ovr_tier;
            w_slot     <= wi_slot;
            w_q        <= 3'd0;
            wi_rdy     <= 1'b0;
            if (wi_is_token && !wi_gen_ok)
              st <= ST_DONE;               // stale epoch: just complete
            else
              st <= ST_QSEL;
          end
        end

        // ------------------------------------------------------------
        ST_QSEL: begin
          if (w_q >= QNUM) begin
            st <= ST_DONE;
          end else if (qd[w_q].enable && (qd[w_q].kind != KIND_NONE) &&
                       (!w_is_token || (w_tok_qid == w_q))) begin
            if (!w_is_token)
              ctr[w_q][0] <= ctr[w_q][0] + 32'd1;   // attempted (arrivals)
            w_row       <= 4'd0;
            hash_cnt    <= 4'd0;
            w_is_cms    <= (qd[w_q].kind == KIND_CMS) ||
                           (qd[w_q].kind == KIND_HH);
            w_rows      <= (qd[w_q].kind == KIND_CMS ||
                            qd[w_q].kind == KIND_HH) ? qd[w_q].rows :
                           (qd[w_q].kind == KIND_BLM)  ? {1'b0, qd[w_q].blm_k} : 4'd1;
            st <= ST_HASH;
          end else begin
            w_q <= w_q + 3'd1;
          end
        end

        // ------------------------------------------------------------
        // the hash cone settles over HASH_WAIT cycles (multicycle path);
        // capture it at the final edge, then address from the register
        ST_HASH: begin
          if (hash_cnt == HASH_WAIT[3:0] - 4'd1) begin
            hash_cnt <= 4'd0;
            hash_reg <= cur_hash;
            if (d.kind == KIND_HLL)
              rank_reg <= clz32(cur_hash << d.cols_log2);
            if (w_row == 4'd0)
              fp_reg <= cur_hash[15:0];
            if (d.kind == KIND_BLM)
              bit_reg <= bit_reg_c;
            st <= ST_LU;
          end else
            hash_cnt <= hash_cnt + 4'd1;
        end

        // ------------------------------------------------------------
        ST_LU: begin
          // replay override: this block is the one being migrated
          if (w_is_token && (w_q == w_ovr_qid) && (cur_blk == w_ovr_blk)) begin
            r_tier[w_row]   <= w_ovr_tier;
            r_addr[w_row]   <= w_ovr_addr;
            r_wib[w_row]    <= cur_wib;
            r_strb[w_row]   <= cur_strb;
            r_bshift[w_row] <= cur_bshift;
            r_blk[w_row]    <= cur_blk;
            st <= ST_LU_ADV;
          end else begin
            dr_val <= 1'b1;
            dr_idx <= {w_q, cur_blk[QBLK_AW-1:0]};
            st <= ST_LUW;
          end
        end

        ST_LUW: begin
          if (dr_val && dr_rdy)
            dr_val <= 1'b0;
          else if (dr_val)
            dr_val <= 1'b1;                      // hold under contention
          if (dr_dv) begin
            if (!dr_ent.valid) begin
              ctr[w_q][1] <= ctr[w_q][1] + 32'd1;   // reject (defensive)
              st <= ST_NEXTQ;
            end else if (dr_ent.owner != OWN_RESIDENT) begin
              r_fslot <= dr_ent.mslot;              // frozen -> divert
              st <= ST_DIVERT;
            end else begin
              r_tier[w_row]   <= dr_ent.tier;
              r_addr[w_row]   <= dr_ent.addr;
              r_wib[w_row]    <= cur_wib;
              r_strb[w_row]   <= cur_strb;
              r_bshift[w_row] <= cur_bshift;
              r_blk[w_row]    <= cur_blk;
              st <= ST_LU_ADV;
            end
          end
        end

        ST_LU_ADV: begin
          if (w_is_cms) begin
            if (w_row + 4'd1 < w_rows) begin
              w_row <= w_row + 4'd1;
              st    <= ST_HASH;
            end else begin
              w_row <= 4'd0;
              st    <= ST_RD;
            end
          end else begin
            st <= ST_RD;
          end
        end

        // ------------------------------------------------------------
        ST_DIVERT: begin
          if (div_full[r_fslot]) begin
            div_val     <= 1'b0;
            ctr[w_q][1] <= ctr[w_q][1] + 32'd1;    // admission reject (Eq. 1)
            st <= ST_NEXTQ;
          end else begin
            div_val  <= 1'b1;
            div_slot <= r_fslot;
            div_tok  <= '{key: w_key, kind: d.kind, qid: w_q, param: 20'h0};
            if (div_val && div_rdy) begin
              div_val    <= 1'b0;
              ctr[w_q][3] <= ctr[w_q][3] + 32'd1;  // diverted (admitted)
              st <= ST_NEXTQ;
            end
          end
        end

        // ------------------------------------------------------------
        ST_RD: begin
          if (r_tier[w_row] == TIER_L0) begin
            l0r_val  <= 1'b1;
            l0r_addr <= l0_word_addr[L0_AW-1:0];
            st <= ST_RDW;
          end else begin
            l1_arvalid <= 1'b1;
            l1_araddr  <= {l1_byte_addr[31:3], 3'b000};
            l1_rready  <= 1'b1;
            st <= ST_L1RD;
          end
        end

        ST_RDW: begin
          if (l0r_val && l0r_rdy)
            l0r_val <= 1'b0;
          else if (l0r_val)
            l0r_val <= 1'b1;                      // hold under contention
          if (l0r_dv) begin
            r_val[w_row] <= cell_ext(l0r_data, d.width, r_bshift[w_row]);
            st <= ST_RD_ADV;
          end
        end

        ST_L1RD: begin
          if (l1_arvalid && l1_arready)
            l1_arvalid <= 1'b0;
          else if (l1_arvalid)
            l1_arvalid <= 1'b1;
          if (l1_rvalid) begin
            l1_rready   <= 1'b0;
            r_val[w_row] <= cell_ext(lane ? l1_rdata[63:32] : l1_rdata[31:0],
                                     d.width, r_bshift[w_row]);
            st <= ST_RD_ADV;
          end
        end

        ST_RD_ADV: begin
          if (w_is_cms) begin
            if (w_row + 4'd1 < w_rows) begin
              w_row <= w_row + 4'd1;
              st    <= ST_RD;
            end else begin
              w_row <= 4'd0;
              st    <= ST_COMPUTE;
            end
          end else begin
            st <= ST_COMPUTE;
          end
        end

        // ------------------------------------------------------------
        // pipeline stage 1: capture the min tree (CMS/HH); everything
        // downstream of r_val is registered before the add/saturate stage
        ST_COMPUTE: begin
          min_reg <= min_c;
          if (w_is_cms)
            w_row <= 4'd0;      // phased write loop starts at row 0
          st <= ST_UPD;
        end

        // pipeline stage 2: +1 / saturate / rank-max / bit-set
        ST_UPD: begin
          case (d.kind)
            KIND_CMS, KIND_HH: begin
              // conservative Count-Min: v = min(cells) + 1
              if (min_reg + 32'd1 > wmax(d.width)) begin
                upd_val    <= wmax(d.width);
                ctr[w_q][5] <= ctr[w_q][5] + 32'd1;   // saturation guard
              end else begin
                upd_val <= min_reg + 32'd1;
              end
            end
            KIND_HLL: begin
              upd_val <= (r_val[w_row] < {24'h0, rank_reg})
                         ? {24'h0, rank_reg} : r_val[w_row];
            end
            KIND_ENT: begin
              if (r_val[w_row] + 32'd1 > wmax(d.width)) begin
                upd_val    <= wmax(d.width);
                ctr[w_q][5] <= ctr[w_q][5] + 32'd1;
              end else begin
                upd_val <= r_val[w_row] + 32'd1;
              end
            end
            default: begin  // KIND_BLM: set the bit
              upd_val <= r_val[w_row] | (32'd1 << bit_reg);
            end
          endcase
          st <= ST_WR;
        end

        // ------------------------------------------------------------
        ST_WR: begin
          if (r_tier[w_row] == TIER_L0) begin
            l0w_val  <= 1'b1;
            l0w_addr <= l0_word_addr[L0_AW-1:0];
            l0w_data <= wr_word;
            l0w_strb <= r_strb[w_row];
            st <= ST_WR_ADV;
          end else begin
            l1_awvalid <= 1'b1;
            l1_awaddr  <= {l1_byte_addr[31:3], 3'b000};
            l1_awdata  <= lane ? {wr_word, 32'h0} : {32'h0, wr_word};
            l1_awstrb  <= lane ? {r_strb[w_row], 4'h0} : {4'h0, r_strb[w_row]};
            st <= ST_L1WR;
          end
        end

        ST_L1WR: begin
          if (l1_awvalid && l1_awready) begin
            l1_awvalid <= 1'b0;
            l1_bready  <= 1'b1;
            st <= ST_L1WR_W;
          end else begin
            l1_awvalid <= 1'b1;                     // hold
          end
        end

        ST_L1WR_W: begin
          if (l1_bvalid) begin
            l1_bready <= 1'b0;
            st <= ST_WR_ADV;
          end
        end

        // shared post-write advance (holds the L0 write under contention)
        ST_WR_ADV: begin
          if (l0w_val && !l0w_rdy) begin
            l0w_val <= 1'b1;
          end else begin
            l0w_val <= 1'b0;
            if (w_is_cms) begin
              if (w_row + 4'd1 < w_rows) begin
                w_row <= w_row + 4'd1;
                st    <= ST_WR;
              end else begin
                ht_val <= 1'b1;
                ht_idx <= {w_q, heat_blk[QBLK_AW-1:0]};
                st     <= ST_HEAT;
              end
            end else begin
              if (w_row + 4'd1 < w_rows) begin
                w_row <= w_row + 4'd1;
                st    <= ST_HASH;      // next bloom hash
              end else begin
                ht_val <= 1'b1;
                ht_idx <= {w_q, heat_blk[QBLK_AW-1:0]};
                st     <= ST_HEAT;
              end
            end
          end
        end

        // ------------------------------------------------------------
        ST_HEAT: begin
          if (ht_val && ht_rdy) begin
            ht_val <= 1'b0;
            if (!w_is_token)
              ctr[w_q][2] <= ctr[w_q][2] + 32'd1;   // direct (applied)
            else
              ctr[w_q][4] <= ctr[w_q][4] + 32'd1;   // replay applied
            rot_ctr <= rot_ctr + 2'd1;
            if (d.kind == KIND_HH)
              st <= ST_HH;
            else
              st <= ST_NEXTQ;
          end else begin
            ht_val <= 1'b1;                          // hold
          end
        end

        // ------------------------------------------------------------
        ST_HH: begin
          if (hh_hit) begin
            if (upd_val > c_cnt[w_q][hh_hit_i])
              c_cnt[w_q][hh_hit_i] <= upd_val;
          end else if (upd_val > hh_min_v) begin
            c_valid[w_q][hh_min_i] <= 1'b1;
            c_fp[w_q][hh_min_i]    <= fp_reg;
            c_cnt[w_q][hh_min_i]   <= upd_val;
          end
          st <= ST_NEXTQ;
        end

        // ------------------------------------------------------------
        ST_NEXTQ: begin
          w_q <= w_q + 3'd1;
          st  <= ST_QSEL;
        end

        // ------------------------------------------------------------
        ST_DONE: begin
          wo_val      <= 1'b1;
          wo_slot     <= w_slot;
          wo_is_token <= w_is_token;
          st          <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

endmodule
