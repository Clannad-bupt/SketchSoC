// Typed, versioned state directory (paper Sec. 3.4.2 / 3.6.3 / 3.7).
//
// Maps {query id, logical block index} to a 64-bit entry
//   {valid, tier, owner, generation, migration slot, physical address}
// and is the single authoritative mapping for every live block.
//
// Owner-state transitions implement the single-writer protocol:
//   RESIDENT -> MIGRATING   directory CAS by the migration engine (freeze)
//   MIGRATING -> REPLAYING  engine snapshot of the replay FIFO
//   REPLAYING -> RESIDENT   final CAS publishes new tier/address/generation
//   MIGRATING/REPLAYING -> RESIDENT (abort: retained tokens drained back)
//
// The module itself is a dumb dual-port BRAM (1 read port + 1 write port);
// compare-and-swap is realised by the owning FSM as read -> check -> write,
// safe because exactly one component (engine or control plane) may mutate
// a given entry at a time -- the paper's single-writer invariant.
//
// Index layout: {qid[QID_W-1:0], block[QBLK_AW-1:0]}.  A query whose
// footprint exceeds 2^QBLK_AW blocks is rejected at install time by the
// control plane (bounded resource allocation, paper Sec. 3.4 / Table 2).
//
// Read requestors (round-robin): 0 = SMU engine, 1 = migration engine,
// 2 = tier-manager scanner, 3 = control plane (remove-walk / dir readback).
// Write requestors: 0 = migration engine, 1 = control plane (install/free).

`timescale 1ns/1ps

module state_directory #(
  parameter int unsigned QBLK_AW = 12     // block index bits per query
)(
  input  logic clk,
  input  logic rst_n,

  // ---- read requestor 0: SMU execution engine -------------------------
  input  logic                r0_val,
  output logic                r0_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] r0_idx,
  output sketchsoc_pkg::dirent_t r0_ent,
  output logic                r0_dv,

  // ---- read requestor 1: migration engine ------------------------------
  input  logic                r1_val,
  output logic                r1_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] r1_idx,
  output sketchsoc_pkg::dirent_t r1_ent,
  output logic                r1_dv,

  // ---- read requestor 2: tier-manager scanner --------------------------
  input  logic                r2_val,
  output logic                r2_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] r2_idx,
  output sketchsoc_pkg::dirent_t r2_ent,
  output logic                r2_dv,

  // ---- read requestor 3: control plane -----------------------------------
  input  logic                r3_val,
  output logic                r3_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] r3_idx,
  output sketchsoc_pkg::dirent_t r3_ent,
  output logic                r3_dv,

  // ---- write requestor 0: migration engine -----------------------------
  input  logic                w0_val,
  output logic                w0_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] w0_idx,
  input  sketchsoc_pkg::dirent_t w0_ent,

  // ---- write requestor 1: control plane --------------------------------
  input  logic                w1_val,
  output logic                w1_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] w1_idx,
  input  sketchsoc_pkg::dirent_t w1_ent
);

  import sketchsoc_pkg::*;

  localparam int unsigned IDXW  = QID_W + QBLK_AW;
  localparam int unsigned DEPTH = 1 << IDXW;

  dirent_t mem [0:DEPTH-1];

  initial begin
    for (int unsigned i = 0; i < DEPTH; i++)
      mem[i] = '0;
  end

  // ---------------- read port: 4-way round robin ------------------------
  // Strict rotation: after serving requestor k the next preference is
  // k+1 mod 4.  (The previous 3-way chain was asymmetric; do not extend
  // it pattern-wise -- rewrite as a uniform rotation when touching it.)
  logic [1:0] rr_rd;                       // next requestor to prefer
  logic r0_g, r1_g, r2_g, r3_g;

  always_comb begin
    r0_g = 1'b0; r1_g = 1'b0; r2_g = 1'b0; r3_g = 1'b0;
    case (rr_rd)
      2'd0: begin
        if (r0_val)      r0_g = 1'b1;
        else if (r1_val) r1_g = 1'b1;
        else if (r2_val) r2_g = 1'b1;
        else if (r3_val) r3_g = 1'b1;
      end
      2'd1: begin
        if (r1_val)      r1_g = 1'b1;
        else if (r2_val) r2_g = 1'b1;
        else if (r3_val) r3_g = 1'b1;
        else if (r0_val) r0_g = 1'b1;
      end
      2'd2: begin
        if (r2_val)      r2_g = 1'b1;
        else if (r3_val) r3_g = 1'b1;
        else if (r0_val) r0_g = 1'b1;
        else if (r1_val) r1_g = 1'b1;
      end
      default: begin
        if (r3_val)      r3_g = 1'b1;
        else if (r0_val) r0_g = 1'b1;
        else if (r1_val) r1_g = 1'b1;
        else if (r2_val) r2_g = 1'b1;
      end
    endcase
  end

  assign r0_rdy = r0_g;
  assign r1_rdy = r1_g;
  assign r2_rdy = r2_g;
  assign r3_rdy = r3_g;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) rr_rd <= 2'd0;
    else if (r0_g) rr_rd <= 2'd1;
    else if (r1_g) rr_rd <= 2'd2;
    else if (r2_g) rr_rd <= 2'd3;
    else if (r3_g) rr_rd <= 2'd0;
  end

  logic            rd_en;
  logic [IDXW-1:0] rd_idx;
  logic [1:0]      rd_who, rd_who_q;
  logic            rd_en_d;

  assign rd_en  = r0_g | r1_g | r2_g | r3_g;
  assign rd_idx = r3_g ? r3_idx : r2_g ? r2_idx : r1_g ? r1_idx : r0_idx;
  assign rd_who = r3_g ? 2'd3 : r2_g ? 2'd2 : r1_g ? 2'd1 : 2'd0;

  dirent_t rd_q;
  always_ff @(posedge clk) begin
    if (rd_en) rd_q <= mem[rd_idx];
  end
  always_ff @(posedge clk) begin
    rd_en_d  <= rd_en;
    rd_who_q <= rd_who;
  end

  assign r0_ent = rd_q;
  assign r1_ent = rd_q;
  assign r2_ent = rd_q;
  assign r3_ent = rd_q;
  // dv qualified by "a read happened": without rd_en_d the idle value of
  // rd_who_q (0) would hold r0_dv high every idle cycle.
  assign r0_dv  = rd_en_d & (rd_who_q == 2'd0);
  assign r1_dv  = rd_en_d & (rd_who_q == 2'd1);
  assign r2_dv  = rd_en_d & (rd_who_q == 2'd2);
  assign r3_dv  = rd_en_d & (rd_who_q == 2'd3);

  // ---------------- write port: 2-way round robin -----------------------
  logic prefer_w1;
  logic w0_g, w1_g;
  assign w0_g = w0_val & ( ~w1_val | ~prefer_w1);
  assign w1_g = w1_val & ( ~w0_val |  prefer_w1);
  assign w0_rdy = w0_g;
  assign w1_rdy = w1_g;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) prefer_w1 <= 1'b0;
    else if (w1_g) prefer_w1 <= 1'b0;
    else if (w0_g) prefer_w1 <= 1'b1;
  end

  logic            wr_en;
  logic [IDXW-1:0] wr_idx;

  assign wr_en  = w0_g | w1_g;
  assign wr_idx = w1_g ? w1_idx : w0_idx;

  always_ff @(posedge clk) begin
    if (wr_en) mem[wr_idx] <= w1_g ? w1_ent : w0_ent;
  end

endmodule
