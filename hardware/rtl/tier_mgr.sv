// Tier manager (paper Sec. 3.6): heat-driven L0/L1 block placement.
//
// Round-robin walk over every (query, block) entry:
//   1. directory read (r2 port)            -> entry
//   2. heat probe with periodic decay      -> 16-bit EWMA heat
//   3. RESIDENT + heat > PROM_TH + L1     -> issue PROMOTE with a free L0
//      RESIDENT + heat < DEM_TH + L0     -> issue DEMOTE with a free L1
//
// Destinations come from per-tier free stacks; the migration engine's
// release port pushes the freed source block back.  Quota exhaustion is
// self-limiting: no destination block == no command, the entry keeps its
// tier and the next scan pass re-evaluates.
//
// The command is held on cmd_val until accepted (the engine only asserts
// cmd_rdy while idle in this serialised v1), so no command queue is
// needed here.  While the engine is mid-migration the scan skips ahead
// -- reading the directory would race the engine's CAS.

`timescale 1ns/1ps

module tier_mgr import sketchsoc_pkg::*; #(
  parameter int unsigned QBLK_AW   = 12,
  parameter int unsigned PROM_TH   = 2048,
  parameter int unsigned DEM_TH    = 128
)(
  input  logic clk,
  input  logic rst_n,

  input  logic         scn_en,

  // ---- directory read requestor 2 ----------------------------------------
  output logic                     dr_val,
  input  logic                     dr_rdy,
  output logic [QID_W+QBLK_AW-1:0] dr_idx,
  input  dirent_t                  dr_ent,
  input  logic                     dr_dv,

  // ---- heat scan port ------------------------------------------------------
  output logic                     ht_val,
  input  logic                     ht_rdy,
  output logic [QID_W+QBLK_AW-1:0] ht_idx,
  output logic                     ht_decay,
  input  logic [15:0]              ht_heat,
  input  logic                     ht_dv,

  // ---- migration command ---------------------------------------------------
  output logic     cmd_val,
  input  logic     cmd_rdy,
  output mig_cmd_t cmd,
  input  logic     mig_busy,

  // ---- source blocks return after each commit ------------------------------
  input  logic        rel_val,
  input  logic [1:0]  rel_tier,
  input  logic [31:0] rel_addr,

  // ---- free-stack management from the control plane -------------------------
  // CONSTRAINT: push_val must never coincide with rel_val -- the shared
  // write path can only retire one push per cycle (rel wins, push_addr is
  // dropped).  The control plane defers its pushes until !rel_val.
  input  logic        push_val,    // seed the pool (not used on the wire)
  input  logic [1:0]  push_tier,
  input  logic [31:0] push_addr,
  input  logic        flush_stacks, // control-plane pool reset: clear both stacks

  // ---- control-plane pop (query install allocates blocks) -------------------
  // Granted only while the scan cannot be latching a destination itself
  // (S_DECIDE reads st_lX[n-1] into cmd_q; S_CMD holds cmd_val), so the
  // same slot can never be handed to both the scanner and the control
  // plane.  pop_addr is valid combinationally in the grant cycle.
  input  logic        pop_val,
  input  logic [1:0]  pop_tier,
  output logic        pop_rdy,
  output logic [31:0] pop_addr,
  output logic        pop_empty,

  // ---- telemetry -------------------------------------------------------------
  output logic [31:0] l0_free_cnt,
  output logic [31:0] l1_free_cnt,
  output logic [31:0] scan_passes,
  output logic [31:0] prom_cnt,
  output logic [31:0] dem_cnt
);

  localparam int unsigned NBLKS = (1 << QBLK_AW);

  // ------------------------------------------------------------------
  // free stacks: LIFO of block base addresses (L0: word base, L1: byte)
  // ------------------------------------------------------------------
  logic [31:0] st_l0 [0:255];
  logic [31:0] st_l1 [0:255];
  logic [8:0]  n_l0;
  logic [8:0]  n_l1;

  assign l0_free_cnt = {23'd0, n_l0};
  assign l1_free_cnt = {23'd0, n_l1};

  // ------------------------------------------------------------------
  // scan pointer + decision registers
  // ------------------------------------------------------------------
  logic [2:0]          scn_q;
  logic [QBLK_AW-1:0]  scn_b;
  logic [2:0]          d_q;
  logic [QBLK_AW-1:0]  d_b;
  dirent_t             d_ent;
  logic [15:0]         d_heat;

  typedef enum logic [3:0] {
    S_IDLE, S_DRW, S_HT, S_HTW, S_DECIDE, S_CMD
  } state_e;
  state_e st;

  mig_cmd_t cmd_q;
  assign cmd = cmd_q;

  logic [31:0] scan_passes_r, prom_cnt_r, dem_cnt_r;
  assign scan_passes = scan_passes_r;
  assign prom_cnt    = prom_cnt_r;
  assign dem_cnt     = dem_cnt_r;

  // advance the scan pointer (shared by S_NEXT and S_SKIP)
  task automatic adv_ptr();
    begin
      if (scn_b == NBLKS-1) begin
        scn_b <= '0;
        if (scn_q == QNUM-1) begin
          scn_q <= 3'd0;
          scan_passes_r <= scan_passes_r + 32'd1;
        end else
          scn_q <= scn_q + 3'd1;
      end else
        scn_b <= scn_b + 1'b1;
    end
  endtask

  // ------------------------------------------------------------------
  // main FSM
  // ------------------------------------------------------------------
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st        <= S_IDLE;
      dr_val    <= 1'b0;
      dr_idx    <= '0;
      ht_val    <= 1'b0;
      ht_idx    <= '0;
      ht_decay  <= 1'b0;
      d_q       <= 3'd0;
      d_b       <= '0;
      d_ent     <= '0;
      d_heat    <= 16'd0;
      cmd_val   <= 1'b0;
      cmd_q     <= '0;
      scn_q     <= 3'd0;
      scn_b     <= '0;
      scan_passes_r <= 32'd0;
      prom_cnt_r    <= 32'd0;
      dem_cnt_r     <= 32'd0;
    end else begin
      dr_val   <= 1'b0;
      ht_val   <= 1'b0;
      ht_decay <= 1'b0;

      case (st)
        S_IDLE: if (scn_en && !mig_busy) begin
          dr_val <= 1'b1;
          dr_idx <= {scn_q, scn_b};
          d_q    <= scn_q;
          d_b    <= scn_b;
          st     <= S_DRW;
        end else if (scn_en) begin  // mig_busy: skip this entry
          adv_ptr();
          st <= S_IDLE;
        end

        S_DRW: begin
          if (dr_val && dr_rdy) dr_val <= 1'b0;
          else                  dr_val <= 1'b1;
          if (dr_dv) begin
            d_ent <= dr_ent;
            st    <= S_HT;
          end
        end

        S_HT: begin
          ht_val   <= 1'b1;
          ht_idx   <= {d_q, d_b};
          ht_decay <= 1'b1;
          st       <= S_HTW;
        end

        S_HTW: begin
          if (ht_val && ht_rdy) begin
            ht_val   <= 1'b0;
            ht_decay <= 1'b0;
          end else begin
            ht_val   <= 1'b1;
            ht_decay <= 1'b1;
          end
          if (ht_dv) begin
            d_heat <= ht_heat;
            st     <= S_DECIDE;
          end
        end

        S_DECIDE: begin
          cmd_q.op    <= MIG_PROMOTE;
          cmd_q.qid   <= d_q;
          cmd_q.block <= {{(16-QBLK_AW){1'b0}}, d_b};
          if (d_ent.valid && (d_ent.owner == OWN_RESIDENT)) begin
            if ((d_ent.tier == TIER_L1) && (d_heat > PROM_TH[15:0]) &&
                (n_l0 != 0)) begin
              cmd_q.op        <= MIG_PROMOTE;
              cmd_q.dest_addr <= st_l0[n_l0 - 8'd1];
              cmd_val         <= 1'b1;
              st              <= S_CMD;
            end else if ((d_ent.tier == TIER_L0) && (d_heat < DEM_TH[15:0]) &&
                         (n_l1 != 0)) begin
              cmd_q.op        <= MIG_DEMOTE;
              cmd_q.dest_addr <= st_l1[n_l1 - 8'd1];
              cmd_val         <= 1'b1;
              st              <= S_CMD;
            end else begin
              adv_ptr();
              st <= S_IDLE;
            end
          end else begin
            adv_ptr();
            st <= S_IDLE;
          end
        end

        S_CMD: begin
          cmd_val <= 1'b1;
          if (cmd_val && cmd_rdy) begin
            cmd_val <= 1'b0;
            if (cmd_q.op == MIG_PROMOTE) prom_cnt_r <= prom_cnt_r + 32'd1;
            else                        dem_cnt_r  <= dem_cnt_r  + 32'd1;
            adv_ptr();
            st <= S_IDLE;
          end
        end

        default: st <= S_IDLE;
      endcase
    end
  end

  // ------------------------------------------------------------------
  // free-stack updates: issue pops, control-plane pops, release/control
  // pushes, capped at the stack depth (overflow is a telemetry event,
  // never a crash).
  // ------------------------------------------------------------------
  wire issue_prom = cmd_val && cmd_rdy && (cmd_q.op == MIG_PROMOTE);
  wire issue_dem  = cmd_val && cmd_rdy && (cmd_q.op == MIG_DEMOTE);
  wire rel_l0 = rel_val && (rel_tier == TIER_L0);
  wire rel_l1 = rel_val && (rel_tier == TIER_L1);
  wire push_l0 = push_val && (push_tier == TIER_L0);
  wire push_l1 = push_val && (push_tier == TIER_L1);

  // control-plane pop: mutually exclusive with the scan's own issue by
  // the state gate (S_DECIDE/S_CMD are the only states that read or hold
  // a stack-derived destination).
  wire pop_l0 = pop_val && (pop_tier == TIER_L0);
  wire pop_l1 = pop_val && (pop_tier == TIER_L1);
  wire pop_qiet = (st != S_DECIDE) && (st != S_CMD);
  wire pop_gnt_l0 = pop_l0 && pop_qiet && (n_l0 != 9'd0);
  wire pop_gnt_l1 = pop_l1 && pop_qiet && (n_l1 != 9'd0);

  assign pop_rdy   = pop_gnt_l0 || pop_gnt_l1;
  assign pop_empty = (pop_tier == TIER_L0) ? (n_l0 == 9'd0) : (n_l1 == 9'd0);
  // guarded top-of-stack read (n==0 would index 511 / wrap)
  assign pop_addr  = (pop_tier == TIER_L0)
                   ? ((n_l0 == 9'd0) ? 32'h0 : st_l0[n_l0[7:0] - 8'd1])
                   : ((n_l1 == 9'd0) ? 32'h0 : st_l1[n_l1[7:0] - 8'd1]);

  // one merged pop term per stack drives BOTH the count case and the
  // write-index selector: pop+push in the same cycle must land the push
  // in the just-freed top slot (n-1), not above it (n).
  wire issue_l0_any = issue_prom || pop_gnt_l0;
  wire issue_l1_any = issue_dem  || pop_gnt_l1;

  wire rel_or_push_l0 = (rel_l0 || push_l0) && (n_l0 < 9'd255);
  wire rel_or_push_l1 = (rel_l1 || push_l1) && (n_l1 < 9'd255);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      n_l0 <= 9'd0;
      n_l1 <= 9'd0;
    end else if (flush_stacks) begin
      // control-plane pool reset: discard every tracked slot.  Only legal
      // while the engine is idle (a committed-but-not-released migration
      // would orphan its source block otherwise); the control plane holds
      // this low during normal operation.
      n_l0 <= 9'd0;
      n_l1 <= 9'd0;
    end else begin
      // separate state updates per possible event pair; the engine only
      // releases after a commit, and the tier manager only issues after the
      // engine is idle, so an issue and release may only meet in the same
      // cycle when the previous command completed in the cycle before --
      // counting handles both without a priority choice.
      case ({issue_l0_any, rel_or_push_l0})
        2'b10: n_l0 <= n_l0 - 9'd1;
        2'b01: n_l0 <= n_l0 + 9'd1;
        2'b11: n_l0 <= n_l0;         // balanced pop+push, top replaced below
        default: n_l0 <= n_l0;
      endcase
      if (rel_or_push_l0)
        st_l0[ (issue_l0_any && rel_or_push_l0) ? (n_l0 - 9'd1) : n_l0 ]
             <= rel_l0 ? rel_addr : push_addr;

      case ({issue_l1_any, rel_or_push_l1})
        2'b10: n_l1 <= n_l1 - 9'd1;
        2'b01: n_l1 <= n_l1 + 9'd1;
        2'b11: n_l1 <= n_l1;
        default: n_l1 <= n_l1;
      endcase
      if (rel_or_push_l1)
        st_l1[ (issue_l1_any && rel_or_push_l1) ? (n_l1 - 9'd1) : n_l1 ]
             <= rel_l1 ? rel_addr : push_addr;
    end
  end

endmodule
