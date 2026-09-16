// Work-item mux (packet ingress vs. migration replay), paper Sec. 3.6.3.
//
// The SMU engine takes one work item at a time.  Replay tokens have priority
// over ingress packets so a migration finishes its passes promptly (bounded
// by the 32-entry per-slot FIFOs); ingress is never head-of-line blocked --
// backpressure propagates through ing_rdy.
//
// req_quiesce (from the migration engine, asserted around a freeze or commit
// directory CAS) stops new ingress grants; the engine itself stops issuing
// replay at the same time, so smu_exec drains to idle and the CAS sees a
// quiescent execution engine.  Replay is not gated: the engine only asserts
// req_quiesce when it has no replay in flight.

`timescale 1ns/1ps

module work_mux import sketchsoc_pkg::*; (
  input  logic clk,
  input  logic rst_n,

  // ---- ingress (packet path) --------------------------------------------
  input  logic        ing_val,
  output logic        ing_rdy,
  input  pkt_desc_t   ing_pkt,
  input  logic [3:0]  ing_slot,      // >= MIG_SLOTS (ingress tag)

  // ---- replay (migration engine) ----------------------------------------
  input  logic        rp_val,
  output logic        rp_rdy,
  input  token_t      rp_tok,
  input  logic        rp_gen_ok,
  input  logic [2:0]  rp_ovr_qid,
  input  logic [15:0] rp_ovr_blk,
  input  logic [31:0] rp_ovr_addr,
  input  logic [1:0]  rp_ovr_tier,
  input  logic [3:0]  rp_slot,

  // ---- quiesce request from the migration engine ------------------------
  input  logic        req_quiesce,

  // ---- to SMU execution engine -------------------------------------------
  output logic        wi_val,
  input  logic        wi_rdy,
  output pkt_desc_t   wi_pkt,
  output logic        wi_is_token,
  output token_t      wi_token,
  output logic        wi_gen_ok,
  output logic [2:0]  wi_ovr_qid,
  output logic [15:0] wi_ovr_blk,
  output logic [31:0] wi_ovr_addr,
  output logic [1:0]  wi_ovr_tier,
  output logic [3:0]  wi_slot
);

  assign rp_rdy  = rp_val & wi_rdy;
  assign ing_rdy = ~rp_val & ~req_quiesce & wi_rdy;

  assign wi_val     = rp_val | (ing_val & ~req_quiesce);
  assign wi_pkt     = ing_pkt;
  assign wi_is_token = rp_val;
  assign wi_token   = rp_tok;
  assign wi_gen_ok  = rp_val & rp_gen_ok;
  assign wi_ovr_qid = rp_val ? rp_ovr_qid : 3'd0;
  assign wi_ovr_blk = rp_val ? rp_ovr_blk : 16'd0;
  assign wi_ovr_addr = rp_val ? rp_ovr_addr : 32'd0;
  assign wi_ovr_tier = rp_val ? rp_ovr_tier : TIER_L0;
  assign wi_slot    = rp_val ? rp_slot : ing_slot;

endmodule
