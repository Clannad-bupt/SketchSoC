// Per-block heat table (paper Sec. 3.4.2 / Fig. 6): a 16-bit EWMA of the
// update rate for every logical block.
//
//   on admitted update : H <= H + INCR - (H >> ASHIFT)
//   on scanner decay   : H <= H >> 1        (one halving per scan pass)
//
// The access update realises Hnew = alpha*Hold + (1-alpha)*U with
// alpha = 1 - 2^-ASHIFT; ASHIFT=3 gives alpha = 0.875, the fixed-point
// neighbour of the paper's example alpha = 0.85.  The periodic halving
// during the tier-manager scan provides the time decay that lets cold
// blocks fall below the demotion floor.
//
// Value range is naturally bounded: the update converges to
// H_eq = INCR << ASHIFT (= 4096 for the defaults) and never exceeds it
// by more than INCR, so no saturation logic is needed.
//
// One operation (access update or scan probe) is granted at a time; each
// is a 2-cycle read-modify-write, which matches the SMU lookup rate.

`timescale 1ns/1ps

module heat_table #(
  parameter int unsigned QBLK_AW = 12,
  parameter int unsigned ASHIFT  = 3,
  parameter int unsigned INCR    = 512
)(
  input  logic clk,
  input  logic rst_n,

  // ---- admitted-update EWMA port (SMU engine) --------------------------
  input  logic                acc_val,
  output logic                acc_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] acc_idx,

  // ---- scan probe port (tier manager) ----------------------------------
  input  logic                scn_val,
  output logic                scn_rdy,
  input  logic [sketchsoc_pkg::QID_W + QBLK_AW - 1:0] scn_idx,
  input  logic                scn_decay,   // halve the entry on probe
  output logic [15:0]         scn_heat,
  output logic                scn_dv       // heat valid the cycle after probe
);

  import sketchsoc_pkg::*;

  localparam int unsigned IDXW  = QID_W + QBLK_AW;
  localparam int unsigned DEPTH = 1 << IDXW;

  logic [15:0] mem [0:DEPTH-1];

  initial begin
    for (int unsigned i = 0; i < DEPTH; i++) mem[i] = 16'h0;
  end

  // ---------------- pipeline registers (declared before use) ------------
  logic            s1_valid;
  logic [IDXW-1:0] s1_idx;
  logic            s1_is_scn;
  logic            s1_decay;
  logic [15:0]     rd_q;

  // ---------------- grant: round robin, ops strictly serialized ---------
  // An operation occupies the RMW for two cycles (read, then write), and a
  // following read of the same index must not observe the pre-write value,
  // so a new operation is granted only when the pipeline is empty.
  logic prefer_scn;
  logic acc_g, scn_g;
  assign acc_g = acc_val & ~s1_valid & (~scn_val | ~prefer_scn);
  assign scn_g = scn_val & ~s1_valid & (~acc_val |  prefer_scn);
  assign acc_rdy = acc_g;
  assign scn_rdy = scn_g;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) prefer_scn <= 1'b0;
    else if (scn_g) prefer_scn <= 1'b0;
    else if (acc_g) prefer_scn <= 1'b1;
  end

  // ---------------- stage 1: read the probed entry ----------------------
  always_ff @(posedge clk) begin
    if (acc_g | scn_g) rd_q <= mem[acc_g ? acc_idx : scn_idx];
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) s1_valid <= 1'b0;
    else begin
      s1_valid  <= acc_g | scn_g;
      s1_idx    <= acc_g ? acc_idx : scn_idx;
      s1_is_scn <= scn_g;
      s1_decay  <= scn_decay;
    end
  end

  // ---------------- stage 2: compute and write back ---------------------
  // rd_q holds the probed value exactly during the s1_valid cycle; a new
  // operation cannot have been granted yet (serialization above).
  logic [15:0] upd_val;
  logic        do_write;

  assign upd_val  = s1_is_scn ? (rd_q >> 1) : (rd_q + INCR - (rd_q >> ASHIFT));
  assign do_write = s1_valid & (~s1_is_scn | s1_decay);

  always_ff @(posedge clk) begin
    if (do_write) mem[s1_idx] <= upd_val;
  end

  // report the post-decay value so the scanner always sees current heat
  assign scn_heat = s1_decay ? (rd_q >> 1) : rd_q;
  assign scn_dv   = s1_valid & s1_is_scn;

endmodule
