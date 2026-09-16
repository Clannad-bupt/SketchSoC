// L0 tier: on-die SRAM (BRAM), 32-bit words, 256-byte state blocks.
//
// Paper Sec. 3.4.1 / Fig. 5: L0 is a "1R1W bank-interleaved" store.  This
// module realises the 1R1W structure as a true-dual-port BRAM (one sync
// read port + one sync write port) shared by two requestors under
// round-robin arbitration:
//
//   requestor 0 : the SMU execution engine (packet + replay updates)
//   requestor 1 : the migration copy engine / control-plane readout
//
// Round-robin (rather than fixed priority) mirrors the paper's NoC
// arbitration, which "reserves credits for every VC" so that memory
// completion cannot be starved by migration traffic or vice versa
// (Sec. 3.9).  A requestor holds its request until granted.

`timescale 1ns/1ps

module l0_store #(
  parameter int unsigned AW = 10               // word address bits (WORDS=2^AW)
)(
  input  logic clk,
  input  logic rst_n,

  // ---- read requestor 0 (SMU execution engine) -----------------------
  input  logic                     r0_val,
  output logic                     r0_rdy,
  input  logic [AW-1:0]            r0_addr,   // word address
  output logic [31:0]              r0_data,
  output logic                     r0_dv,     // data valid, 1 cycle after grant

  // ---- read requestor 1 (migration / control readout) -----------------
  input  logic                     r1_val,
  output logic                     r1_rdy,
  input  logic [AW-1:0]            r1_addr,
  output logic [31:0]              r1_data,
  output logic                     r1_dv,

  // ---- write requestor 0 (SMU execution engine) -----------------------
  input  logic                     w0_val,
  output logic                     w0_rdy,
  input  logic [AW-1:0]            w0_addr,
  input  logic [31:0]              w0_data,
  input  logic [3:0]               w0_strb,   // byte strobes

  // ---- write requestor 1 (migration / control) ------------------------
  input  logic                     w1_val,
  output logic                     w1_rdy,
  input  logic [AW-1:0]            w1_addr,
  input  logic [31:0]              w1_data,
  input  logic [3:0]               w1_strb
);

  // capacity in 256-byte blocks; 64 words per block
  localparam int unsigned WORDS = 1 << AW;
  localparam int unsigned BLKS  = WORDS / 64;

`ifndef SYNTHESIS
  initial begin
    if (BLKS == 0)
      $fatal(1, "l0_store: AW=%0d gives fewer than one 256-byte block", AW);
  end
`endif

  logic [31:0] mem [0:WORDS-1];

  // BRAM initialisation to zero (matches Vivado BRAM power-up state).
  initial begin
    for (int unsigned i = 0; i < WORDS; i++) mem[i] = 32'h0;
  end

  // ---------------- read port arbitration ------------------------------
  logic prefer_r1;                     // round-robin state
  logic r0_g, r1_g;
  assign r0_g = r0_val & ( ~r1_val | ~prefer_r1);
  assign r1_g = r1_val & ( ~r0_val |  prefer_r1);
  assign r0_rdy = r0_g;
  assign r1_rdy = r1_g;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) prefer_r1 <= 1'b0;
    else if (r1_g) prefer_r1 <= 1'b0;
    else if (r0_g) prefer_r1 <= 1'b1;
  end

  logic        rd_en;
  logic [AW-1:0] rd_addr;
  logic        rd_to_r1;
  logic        rd_en_d, rd_to_r1_q;

  assign rd_en   = r0_g | r1_g;
  assign rd_addr = r1_g ? r1_addr : r0_addr;
  assign rd_to_r1 = r1_g;

  logic [31:0] rd_data_q;
  always_ff @(posedge clk) begin
    if (rd_en) rd_data_q <= mem[rd_addr];
  end

  always_ff @(posedge clk) begin
    rd_en_d     <= rd_en;
    rd_to_r1_q  <= rd_to_r1;
  end

  assign r0_data = rd_data_q;
  assign r1_data = rd_data_q;
  assign r0_dv   = rd_en_d & ~rd_to_r1_q;
  assign r1_dv   = rd_en_d &  rd_to_r1_q;

  // ---------------- write port arbitration -----------------------------
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

  logic        wr_en;
  logic [AW-1:0] wr_addr;
  logic [31:0]  wr_data;
  logic [3:0]   wr_strb;

  assign wr_en   = w0_g | w1_g;
  assign wr_addr = w1_g ? w1_addr : w0_addr;
  assign wr_data = w1_g ? w1_data : w0_data;
  assign wr_strb = w1_g ? w1_strb : w0_strb;

  always_ff @(posedge clk) begin
    if (wr_en) begin
      if (wr_strb[0]) mem[wr_addr][7:0]   <= wr_data[7:0];
      if (wr_strb[1]) mem[wr_addr][15:8]  <= wr_data[15:8];
      if (wr_strb[2]) mem[wr_addr][23:16] <= wr_data[23:16];
      if (wr_strb[3]) mem[wr_addr][31:24] <= wr_data[31:24];
    end
  end

endmodule
