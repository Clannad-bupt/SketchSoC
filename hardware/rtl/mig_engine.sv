// Migration engine (paper Sec. 3.6.3): the single-writer block-migration
// protocol that makes tiered state reconfigurable while queries keep
// running.
//
// Per migration (one command from the tier manager):
//
//   FREEZE    quiesce the SMU engine (req_quiesce -> exec idle), then a
//             directory CAS: RESIDENT -> MIGRATING, mslot = slot.  From
//             this point SMU operations touching the block divert as one
//             128-bit logical token into the slot's FIFO (or are rejected
//             and counted if the FIFO is full -- Eq. 1, never a stall).
//   COPY      256-byte block snapshot source -> destination: PROMOTE reads
//             32 x 64-bit L1 beats and writes 64 L0 words; DEMOTE reads 64
//             L0 words and writes 32 L1 beats.  The source is frozen and
//             the destination unpublished, so the copy is race-free.
//   REPLAY    CAS MIGRATING -> REPLAYING (no quiesce needed: both owner
//             states divert, same slot).  Then bounded passes: each pass
//             marks the FIFO prefix, re-executes every token through the
//             SMU engine with a block override {qid, block, dest, tier}
//             (replay re-hashes the key -- the token is an operation, not
//             an address), and pops only on completion.  Tokens that hit
//             another frozen block re-divert (later pass).  A pass that
//             drains with no new arrivals ends the phase; every token is
//             applied exactly once.
//   COMMIT    quiesce, re-check the FIFO (a late divert from the quiesce
//             window runs another pass), then the final CAS publishes
//             {tier = dest, addr = dest, gen + 1, RESIDENT}, frees the
//             slot, and releases the source block to the tier manager.
//
// Epoch guard: replay tokens are dropped (counted) when the owning query's
// configuration generation has moved on since the freeze -- a stale token
// never corrupts reconfigured state.
//
// v1 serialises migrations (one active slot; further commands back-pressure
// through cmd_rdy).  The slot pool and per-slot FIFOs match the paper's
// bounded shared pool so a pipelined engine can later overlap phases.

`timescale 1ns/1ps

module mig_engine import sketchsoc_pkg::*; #(
  parameter int unsigned QBLK_AW = 12,
  parameter int unsigned L0_AW   = 12
)(
  input  logic clk,
  input  logic rst_n,

  // ---- migration command (tier manager) ----------------------------------
  input  logic        cmd_val,
  output logic        cmd_rdy,
  input  mig_cmd_t    cmd,

  // ---- per-query configuration generations (epoch guard) -----------------
  input  logic [7:0]  qgen [QNUM],

  // ---- execution-engine quiescence ----------------------------------------
  input  logic        exec_idle,
  output logic        req_quiesce,

  // ---- directory: read requestor 1, write requestor 0 ---------------------
  output logic                     dr_val,
  input  logic                     dr_rdy,
  output logic [QID_W+QBLK_AW-1:0] dr_idx,
  input  dirent_t                  dr_ent,
  input  logic                     dr_dv,
  output logic                     dw_val,
  input  logic                     dw_rdy,
  output logic [QID_W+QBLK_AW-1:0] dw_idx,
  output dirent_t                  dw_ent,

  // ---- L0 store: read requestor 1 + write requestor 1 ---------------------
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

  // ---- L1 (AXI4-lite subset, 64-bit single beat) ---------------------------
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

  // ---- divert ingest from the SMU engine (per-slot FIFOs live here) -------
  output logic [MIG_SLOTS-1:0] div_full,
  input  logic                 div_val,
  input  logic [3:0]           div_slot,
  input  token_t               div_tok,
  output logic                 div_rdy,

  // ---- replay work out (to work_mux -> smu_exec) --------------------------
  output logic        rp_val,
  input  logic        rp_rdy,
  output token_t      rp_tok,
  output logic        rp_gen_ok,
  output logic [2:0]  rp_ovr_qid,
  output logic [15:0] rp_ovr_blk,
  output logic [31:0] rp_ovr_addr,
  output logic [1:0]  rp_ovr_tier,
  output logic [3:0]  rp_slot,

  // ---- completion from the SMU engine -------------------------------------
  input  logic        wo_val,
  input  logic [3:0]  wo_slot,
  input  logic        wo_is_token,

  // ---- source-block release (tier-manager free list) -----------------------
  output logic        rel_val,
  output logic [1:0]  rel_tier,
  output logic [31:0] rel_addr,

  // ---- status / telemetry --------------------------------------------------
  output logic        busy,
  output logic [31:0] mig_cnt,       // migrations committed
  output logic [31:0] rep_ok_cnt,    // replay tokens applied
  output logic [31:0] rep_drop_cnt,  // replay tokens dropped (stale epoch)
  output logic [31:0] cmd_rej_cnt,   // commands rejected (owner not RESIDENT)
  output logic [31:0] err_cnt        // owner-state protocol violations
);

  localparam int unsigned SLOT_W = (MIG_SLOTS <= 1) ? 1 : $clog2(MIG_SLOTS);

  // ------------------------------------------------------------------
  // FSM
  // ------------------------------------------------------------------
  typedef enum logic [4:0] {
    ST_IDLE,
    ST_FRZ_Q,                    // quiesce wait before freeze CAS
    ST_CAS_RW, ST_CAS_W,         // freeze CAS (read issued at FRZ_Q exit)
    ST_CPR, ST_CPRW,             // copy: first read of the beat
    ST_CPR1, ST_CPR1B, ST_CPR1W, // copy: demote second word read (+bubble)
    ST_CPW0, ST_CPW0W,           // copy: promote L0 word writes
    ST_CPW1, ST_CPW1W,
    ST_CPAW, ST_CPAWW, ST_CPAWX, // copy: demote L1 beat write
    ST_CAS2_RW, ST_CAS2_W, ST_CAS2_W_WAIT,   // MIGRATING -> REPLAYING
    ST_MARK, ST_RP, ST_RPW, ST_RPA,
    ST_CMT_Q,                    // quiesce wait before commit CAS
    ST_CAS3_RW, ST_CAS3_W,       // commit CAS (read issued at CMT_Q exit)
    ST_DRAINCHK,                 // post-commit: drain any commit-window token
    ST_DONE
  } state_e;

  state_e st;
  assign busy = (st != ST_IDLE);

  // command context
  logic [1:0]  op_q;
  logic [2:0]  qid_q;
  logic [15:0] blk_q;
  logic [31:0] dest_q;
  logic [1:0]  dest_tier_q;
  logic        pr_q;                       // promote (L1 -> L0)
  logic [SLOT_W-1:0]     act_slot;
  logic [MIG_SLOTS-1:0]  slot_busy;

  // directory / copy data path
  dirent_t     src_ent;
  logic [5:0]  beat;
  logic [31:0] wA, wB;
  logic        ar_sent;
  logic [7:0]  src_gen_q;                  // config epoch at freeze
  logic [L0_AW-1:0] src_word, dst_word;

  // replay
  logic [6:0]  pass_left;
  logic        rp_gen_q;

  logic [31:0] rep_ok_cnt_r, rep_drop_cnt_r, mig_cnt_r, cmd_rej_cnt_r, err_cnt_r;

  assign src_word = src_ent.addr[L0_AW-1:0];
  assign dst_word = dest_q[L0_AW-1:0];

  // ------------------------------------------------------------------
  // per-slot replay token FIFOs (bounded shared pool)
  // ------------------------------------------------------------------
  token_t     fmem  [0:MIG_SLOTS-1][0:FIFO_DEPTH-1];
  logic [4:0] fhead [0:MIG_SLOTS-1];
  logic [4:0] ftail [0:MIG_SLOTS-1];
  logic [5:0] fcnt  [0:MIG_SLOTS-1];

  always_comb begin
    for (int unsigned i = 0; i < MIG_SLOTS; i++)
      div_full[i] = (fcnt[i] == FIFO_DEPTH);
  end

  // accept a divert unless that slot's FIFO is full (slots outside the pool
  // are accepted and discarded -- cannot happen, mslot is bounded)
  assign div_rdy = (div_slot >= MIG_SLOTS) ? 1'b1 :
                   ~div_full[div_slot[SLOT_W-1:0]];

  token_t head_tok;
  logic   gen_ok_c;
  assign head_tok = fmem[act_slot][fhead[act_slot]];
  assign gen_ok_c = (head_tok.qid < QNUM) &&
                    (qgen[head_tok.qid] == src_gen_q);

  // pop fires on the completion of the outstanding replay token: the same
  // edge as the engine's pass_left decrement, so fcnt is already consistent
  // when ST_RPA decides pass / commit one cycle later.
  logic pop_now;
  assign pop_now = (st == ST_RPW) && wo_val && wo_is_token &&
                   (wo_slot == {{(4-SLOT_W){1'b0}}, act_slot});

  logic [MIG_SLOTS-1:0] fwr, frd;
  always_comb begin
    for (int unsigned i = 0; i < MIG_SLOTS; i++) begin
      fwr[i] = div_val & div_rdy & (div_slot == i);
      frd[i] = pop_now & (act_slot == i);
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int unsigned i = 0; i < MIG_SLOTS; i++) begin
        fhead[i] <= 5'd0;
        ftail[i] <= 5'd0;
        fcnt[i]  <= 6'd0;
      end
    end else begin
      for (int unsigned i = 0; i < MIG_SLOTS; i++) begin
        if (fwr[i]) begin
          fmem[i][ftail[i]] <= div_tok;
          ftail[i] <= ftail[i] + 5'd1;
        end
        if (frd[i])
          fhead[i] <= fhead[i] + 5'd1;
        if (fwr[i] != frd[i])
          fcnt[i] <= fwr[i] ? (fcnt[i] + 6'd1) : (fcnt[i] - 6'd1);
      end
    end
  end

  // ------------------------------------------------------------------
  // slot allocation (v1: at most one active; the pool matches the paper's
  // bounded shared slots)
  // ------------------------------------------------------------------
  logic [MIG_SLOTS-1:0] slot_free;
  logic [SLOT_W-1:0]    act_pick;
  assign slot_free = ~slot_busy;
  always_comb begin
    act_pick = '0;
    for (int unsigned i = MIG_SLOTS; i > 0; i--)
      if (slot_free[i-1]) act_pick = SLOT_W'(i - 1);
  end
  assign cmd_rdy = (st == ST_IDLE) & (|slot_free);

  // ------------------------------------------------------------------
  // main FSM
  // ------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st          <= ST_IDLE;
      slot_busy   <= '0;
      req_quiesce <= 1'b0;
      dr_val      <= 1'b0;
      dw_val      <= 1'b0;
      l0r_val     <= 1'b0;
      l0w_val     <= 1'b0;
      l1_arvalid  <= 1'b0;
      l1_rready   <= 1'b0;
      l1_awvalid  <= 1'b0;
      l1_bready   <= 1'b0;
      rp_val      <= 1'b0;
      rel_val     <= 1'b0;
      beat        <= 6'd0;
      ar_sent     <= 1'b0;
      pass_left   <= 7'd0;
      rp_gen_q    <= 1'b0;
      src_gen_q   <= 8'd0;
      mig_cnt_r      <= 32'd0;
      rep_ok_cnt_r   <= 32'd0;
      rep_drop_cnt_r <= 32'd0;
      cmd_rej_cnt_r  <= 32'd0;
      err_cnt_r      <= 32'd0;
    end else begin
      // one-cycle defaults; states re-assert what they hold
      dr_val     <= 1'b0;
      dw_val     <= 1'b0;
      l0r_val    <= 1'b0;
      l0w_val    <= 1'b0;
      l1_arvalid <= 1'b0;
      l1_rready  <= 1'b0;
      l1_awvalid <= 1'b0;
      l1_bready  <= 1'b0;
      rp_val     <= 1'b0;
      rel_val    <= 1'b0;

      case (st)
        // --------------------------------------------------------------
        ST_IDLE: begin
          if (cmd_val && cmd_rdy) begin
            op_q        <= cmd.op;
            qid_q       <= cmd.qid;
            blk_q       <= cmd.block;
            dest_q      <= cmd.dest_addr;
            pr_q        <= (cmd.op == MIG_PROMOTE);
            dest_tier_q <= (cmd.op == MIG_PROMOTE) ? TIER_L0 : TIER_L1;
            act_slot    <= act_pick;
            slot_busy[act_pick] <= 1'b1;
            beat        <= 6'd0;
            req_quiesce <= 1'b1;
            st          <= ST_FRZ_Q;
          end
        end

        // --------------------------------------------------------------
        // freeze: quiesce, then CAS RESIDENT -> MIGRATING.  req_quiesce
        // rises on the accept edge, so by the time it is sampled here the
        // mux has stopped granting ingress and exec_idle reflects every
        // granted operation.
        // --------------------------------------------------------------
        ST_FRZ_Q: begin
          if (req_quiesce && exec_idle) begin
            dr_val <= 1'b1;
            dr_idx <= {qid_q, blk_q[QBLK_AW-1:0]};
            st     <= ST_CAS_RW;
          end
        end

        ST_CAS_RW: begin
          if (dr_val && dr_rdy) dr_val <= 1'b0;
          else                  dr_val <= 1'b1;
          if (dr_dv) begin
            src_ent   <= dr_ent;
            src_gen_q <= qgen[qid_q];
            if (dr_ent.valid && (dr_ent.owner == OWN_RESIDENT)) begin
              // {valid, tier, MIGRATING, gen, mslot, addr, rsv}
              dw_val <= 1'b1;
              dw_idx <= {qid_q, blk_q[QBLK_AW-1:0]};
              dw_ent <= {1'b1, dr_ent.tier, OWN_MIGRATING, dr_ent.gen,
                         {{(4-SLOT_W){1'b0}}, act_slot}, dr_ent.addr, 15'd0};
              st     <= ST_CAS_W;
            end else begin
              cmd_rej_cnt_r      <= cmd_rej_cnt_r + 32'd1;
              slot_busy[act_slot] <= 1'b0;
              req_quiesce        <= 1'b0;
              st                 <= ST_IDLE;
            end
          end
        end

        ST_CAS_W: begin
          if (dw_val && dw_rdy) begin
            req_quiesce <= 1'b0;
            st          <= ST_CPR;
          end else begin
            dw_val <= 1'b1;
          end
        end

        // --------------------------------------------------------------
        // copy: 256 B = 32 x 64-bit beats = 64 x 32-bit words
        //   promote: L1 beat read -> two L0 word writes
        //   demote  : two L0 word reads -> one L1 beat write
        // --------------------------------------------------------------
        ST_CPR: begin
          if (pr_q) begin
            l1_araddr  <= src_ent.addr + {beat[4:0], 3'b000};
            l1_arvalid <= 1'b1;
            l1_rready  <= 1'b1;
          end else begin
            l0r_addr <= src_word + {beat[4:0], 1'b0};
            l0r_val  <= 1'b1;
          end
          st <= ST_CPRW;
        end

        ST_CPRW: begin
          if (pr_q) begin
            if (!ar_sent) begin
              if (l1_arvalid && l1_arready) begin
                l1_arvalid <= 1'b0;
                ar_sent    <= 1'b1;
              end else begin
                l1_arvalid <= 1'b1;
              end
            end
            l1_rready <= 1'b1;
            if (l1_rvalid && l1_rready) begin
              wA        <= l1_rdata[31:0];
              wB        <= l1_rdata[63:32];
              l1_rready <= 1'b0;
              ar_sent   <= 1'b0;
              st        <= ST_CPW0;
            end
          end else begin
            if (l0r_val && l0r_rdy) l0r_val <= 1'b0;
            else                    l0r_val <= 1'b1;
            if (l0r_dv) begin
              wA <= l0r_data;
              st <= ST_CPR1;
            end
          end
        end

        // demote: second word of the beat ({beat,1} = 2*beat + 1)
        // demote: second word of the beat ({beat,1} = 2*beat + 1).  The
        // prior read's dv clears only on the cycle after that grant; wait
        // one bubble (ST_CPR1B) so a stale dv cannot be mistaken for this
        // request's completion.
        ST_CPR1: begin
          l0r_addr <= src_word + {beat[4:0], 1'b1};
          l0r_val  <= 1'b1;
          st       <= ST_CPR1B;
        end

        ST_CPR1B: begin
          st <= ST_CPR1W;
        end

        ST_CPR1W: begin
          if (l0r_val && l0r_rdy) l0r_val <= 1'b0;
          else                    l0r_val <= 1'b1;
          if (l0r_dv) begin
            wB <= l0r_data;
            st <= ST_CPAW;
          end
        end

        // promote: write the two words to L0
        ST_CPW0: begin
          l0w_addr <= dst_word + {beat[4:0], 1'b0};
          l0w_data <= wA;
          l0w_strb <= 4'hF;
          l0w_val  <= 1'b1;
          st       <= ST_CPW0W;
        end

        ST_CPW0W: begin
          if (l0w_val && l0w_rdy) st <= ST_CPW1;
          else                    l0w_val <= 1'b1;
        end

        ST_CPW1: begin
          l0w_addr <= dst_word + {beat[4:0], 1'b1};
          l0w_data <= wB;
          l0w_strb <= 4'hF;
          l0w_val  <= 1'b1;
          st       <= ST_CPW1W;
        end

        ST_CPW1W: begin
          if (l0w_val && l0w_rdy) begin
            if (beat == 6'd31) st <= ST_CAS2_RW;
            else begin
              beat <= beat + 6'd1;
              st   <= ST_CPR;
            end
          end else begin
            l0w_val <= 1'b1;
          end
        end

        // demote: write the beat to L1
        ST_CPAW: begin
          l1_awaddr  <= dest_q + {beat[4:0], 3'b000};
          l1_awdata  <= {wB, wA};
          l1_awstrb  <= 8'hFF;
          l1_awvalid <= 1'b1;
          st         <= ST_CPAWW;
        end

        ST_CPAWW: begin
          if (l1_awvalid && l1_awready) begin
            l1_awvalid <= 1'b0;
            l1_bready  <= 1'b1;
            st         <= ST_CPAWX;
          end else begin
            l1_awvalid <= 1'b1;
          end
        end

        ST_CPAWX: begin
          l1_bready <= 1'b1;
          if (l1_bvalid && l1_bready) begin
            l1_bready <= 1'b0;
            if (beat == 6'd31) st <= ST_CAS2_RW;
            else begin
              beat <= beat + 6'd1;
              st   <= ST_CPR;
            end
          end
        end

        // --------------------------------------------------------------
        // CAS MIGRATING -> REPLAYING.  No quiesce: both owner states
        // divert to the same slot, so an operation reading either value
        // is safe; the re-read + owner check keeps the CAS honest.
        // --------------------------------------------------------------
        ST_CAS2_RW: begin
          dr_val <= 1'b1;
          dr_idx <= {qid_q, blk_q[QBLK_AW-1:0]};
          st     <= ST_CAS2_W;
        end

        ST_CAS2_W: begin
          if (dr_val && dr_rdy) dr_val <= 1'b0;
          else                  dr_val <= 1'b1;
          if (dr_dv) begin
            if ((dr_ent.owner == OWN_MIGRATING) &&
                (dr_ent.mslot == {{(4-SLOT_W){1'b0}}, act_slot})) begin
              dw_val <= 1'b1;
              dw_idx <= {qid_q, blk_q[QBLK_AW-1:0]};
              dw_ent <= {1'b1, dr_ent.tier, OWN_REPLAYING, dr_ent.gen,
                         {{(4-SLOT_W){1'b0}}, act_slot}, dr_ent.addr, 15'd0};
              st     <= ST_CAS2_W_WAIT;
            end else begin
              err_cnt_r         <= err_cnt_r + 32'd1;
              slot_busy[act_slot] <= 1'b0;
              st                <= ST_IDLE;
            end
          end
        end

        ST_CAS2_W_WAIT: begin
          if (dw_val && dw_rdy) st <= ST_MARK;
          else                  dw_val <= 1'b1;
        end

        // --------------------------------------------------------------
        // replay: bounded passes over the slot FIFO
        // --------------------------------------------------------------
        ST_MARK: begin
          pass_left <= {1'b0, fcnt[act_slot]};
          st <= (fcnt[act_slot] == 6'd0) ? ST_CMT_Q : ST_RP;
        end

        ST_RP: begin
          rp_val      <= 1'b1;
          rp_slot     <= {{(4-SLOT_W){1'b0}}, act_slot};
          rp_ovr_qid  <= qid_q;
          rp_ovr_blk  <= blk_q;
          rp_ovr_addr <= dest_q;
          rp_ovr_tier <= dest_tier_q;
          rp_tok      <= head_tok;
          rp_gen_ok   <= gen_ok_c;
          rp_gen_q    <= gen_ok_c;
          if (rp_val && rp_rdy) begin
            rp_val <= 1'b0;
            st     <= ST_RPW;
          end
        end

        ST_RPW: begin
          if (pop_now) begin
            pass_left <= pass_left - 7'd1;
            if (rp_gen_q) rep_ok_cnt_r   <= rep_ok_cnt_r   + 32'd1;
            else          rep_drop_cnt_r <= rep_drop_cnt_r + 32'd1;
            st <= ST_RPA;
          end
        end

        ST_RPA: begin
          if (pass_left == 7'd0) begin
            // pass drained: new arrivals mean another pass, else commit
            st <= (fcnt[act_slot] != 6'd0) ? ST_MARK : ST_CMT_Q;
          end else begin
            st <= ST_RP;
          end
        end

        // --------------------------------------------------------------
        // commit: quiesce, re-check the FIFO (a divert from the window
        // before quiesce became visible runs another pass), publish.
        // The exec_idle check is gated on req_quiesce already being high,
        // so no operation can be granted after the sampled idle cycle.
        // --------------------------------------------------------------
        ST_CMT_Q: begin
          req_quiesce <= 1'b1;
          if (req_quiesce && exec_idle) begin
            if (fcnt[act_slot] != 6'd0) begin
              req_quiesce <= 1'b0;      // late divert: run another pass
              st          <= ST_MARK;
            end else begin
              dr_val <= 1'b1;
              dr_idx <= {qid_q, blk_q[QBLK_AW-1:0]};
              st     <= ST_CAS3_RW;
            end
          end
        end

        ST_CAS3_RW: begin
          if (dr_val && dr_rdy) dr_val <= 1'b0;
          else                  dr_val <= 1'b1;
          if (dr_dv) begin
            if ((dr_ent.owner == OWN_REPLAYING) &&
                (dr_ent.mslot == {{(4-SLOT_W){1'b0}}, act_slot})) begin
              dw_val  <= 1'b1;
              dw_idx  <= {qid_q, blk_q[QBLK_AW-1:0]};
              dw_ent  <= {1'b1, dest_tier_q, OWN_RESIDENT,
                          dr_ent.gen + 8'd1, 4'd0, dest_q, 15'd0};
              rel_tier <= src_ent.tier;
              rel_addr <= src_ent.addr;
              st       <= ST_CAS3_W;
            end else begin
              err_cnt_r         <= err_cnt_r + 32'd1;
              slot_busy[act_slot] <= 1'b0;
              req_quiesce       <= 1'b0;
              st                <= ST_IDLE;
            end
          end
        end

        ST_CAS3_W: begin
          if (dw_val && dw_rdy) begin
            rel_val <= 1'b1;
            st      <= ST_DRAINCHK;
          end else begin
            dw_val <= 1'b1;
          end
        end

        // Post-commit orphan drain.  A packet whose directory read won the
        // race with the commit CAS can observe the OLD REPLAYING entry and
        // divert one last token after ST_CMT_Q's fcnt==0 check.  The token's
        // override still points at the now-published destination, so a drain
        // pass applies it exactly once.  Release quiesce only after the
        // drain completes, so no outside op can see the in-between state.
        ST_DRAINCHK: begin
          if (fcnt[act_slot] != 6'd0)
            st <= ST_MARK;
          else begin
            req_quiesce <= 1'b0;
            st          <= ST_DONE;
          end
        end

        ST_DONE: begin
          slot_busy[act_slot] <= 1'b0;
          mig_cnt_r           <= mig_cnt_r + 32'd1;
          st                  <= ST_IDLE;
        end

        default: st <= ST_IDLE;
      endcase
    end
  end

  assign mig_cnt      = mig_cnt_r;
  assign rep_ok_cnt   = rep_ok_cnt_r;
  assign rep_drop_cnt = rep_drop_cnt_r;
  assign cmd_rej_cnt  = cmd_rej_cnt_r;
  assign err_cnt      = err_cnt_r;

endmodule
