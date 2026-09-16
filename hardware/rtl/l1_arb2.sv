// Two-master L1 arbiter (AXI4-lite subset, 64-bit single beat).
//
// Master 0 is the SMU execution engine, master 1 the migration engine.
// Directions follow AXI master/slave convention:
//   masters drive arvalid/araddr/awvalid/awaddr/awdata/awstrb/rready/bready
//   this block returns arready/rvalid/rdata/awready/bvalid,
//   and drives the slave side with valid/addr/data/ready transposed.
//
// A read's ar-address accept pins that master; its r-handshake frees it.
// Writes likewise via aw -> b.  Round-robin against the free channel picks
// who gets it; once pinned, only the owner can make progress on that
// channel (preventing interleaved beats on a 1-outstanding slave).
// Masters hold valid until ready, so the arbiter routes, never buffers.

`timescale 1ns/1ps

module l1_arb2 (
  input  logic clk,
  input  logic rst_n,

  // ---- master 0: SMU engine ----------------------------------------------
  input  logic        m0_arvalid,
  output logic        m0_arready,
  input  logic [31:0] m0_araddr,
  output logic        m0_rvalid,
  input  logic        m0_rready,
  output logic [63:0] m0_rdata,
  input  logic        m0_awvalid,
  output logic        m0_awready,
  input  logic [31:0] m0_awaddr,
  input  logic [63:0] m0_awdata,
  input  logic [7:0]  m0_awstrb,
  output logic        m0_bvalid,
  input  logic        m0_bready,

  // ---- master 1: migration engine ----------------------------------------
  input  logic        m1_arvalid,
  output logic        m1_arready,
  input  logic [31:0] m1_araddr,
  output logic        m1_rvalid,
  input  logic        m1_rready,
  output logic [63:0] m1_rdata,
  input  logic        m1_awvalid,
  output logic        m1_awready,
  input  logic [31:0] m1_awaddr,
  input  logic [63:0] m1_awdata,
  input  logic [7:0]  m1_awstrb,
  output logic        m1_bvalid,
  input  logic        m1_bready,

  // ---- slave --------------------------------------------------------------
  output logic        s_arvalid,
  input  logic        s_arready,
  output logic [31:0] s_araddr,
  input  logic        s_rvalid,
  output logic        s_rready,
  input  logic [63:0] s_rdata,
  output logic        s_awvalid,
  input  logic        s_awready,
  output logic [31:0] s_awaddr,
  output logic [63:0] s_awdata,
  output logic [7:0]  s_awstrb,
  input  logic        s_bvalid,
  output logic        s_bready
);

  // ---- read channel -------------------------------------------------------
  logic        rd_pin_v;    // a read is in flight (ar accepted, r pending)
  logic        rd_pin;      // 0 = m0, 1 = m1
  logic        rr_rd;       // round-robin tie preference
  logic        rd_sel_m1;

  // on a free channel, grant ar to whichever master rr points to (or the
  // only one asking); a pinned channel always follows the owner.
  assign rd_sel_m1 = rd_pin_v ? rd_pin
                              : (~m0_arvalid) ? m1_arvalid
                              : (~m1_arvalid) ? 1'b0
                              : rr_rd;

  assign s_arvalid  = (rd_sel_m1 ? m1_arvalid : m0_arvalid);
  assign s_araddr   = rd_sel_m1 ? m1_araddr : m0_araddr;
  assign m0_arready = ~rd_sel_m1 & s_arready;
  assign m1_arready =  rd_sel_m1 & s_arready;

  assign m0_rvalid = s_rvalid & ~rd_sel_m1;
  assign m1_rvalid = s_rvalid &  rd_sel_m1;
  assign m0_rdata  = s_rdata;
  assign m1_rdata  = s_rdata;
  assign s_rready  = rd_sel_m1 ? m1_rready : m0_rready;

  logic rd_fire_ar, rd_fire_r;
  assign rd_fire_ar = s_arvalid & s_arready;
  assign rd_fire_r  = s_rvalid & s_rready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rd_pin_v <= 1'b0;
      rd_pin   <= 1'b0;
      rr_rd    <= 1'b0;
    end else begin
      if (rd_fire_ar) begin
        rd_pin_v <= 1'b1;
        rd_pin   <= rd_sel_m1;
        rr_rd    <= ~rd_sel_m1;      // loser gets preference next time
      end
      if (rd_fire_r) rd_pin_v <= 1'b0;
    end
  end

  // ---- write channel ------------------------------------------------------
  logic        wr_pin_v;
  logic        wr_pin;
  logic        rr_wr;
  logic        wr_sel_m1;

  assign wr_sel_m1 = wr_pin_v ? wr_pin
                              : (~m0_awvalid) ? m1_awvalid
                              : (~m1_awvalid) ? 1'b0
                              : rr_wr;

  assign s_awvalid  = wr_sel_m1 ? m1_awvalid : m0_awvalid;
  assign s_awaddr   = wr_sel_m1 ? m1_awaddr : m0_awaddr;
  assign s_awdata   = wr_sel_m1 ? m1_awdata : m0_awdata;
  assign s_awstrb   = wr_sel_m1 ? m1_awstrb : m0_awstrb;
  assign m0_awready = ~wr_sel_m1 & s_awready;
  assign m1_awready =  wr_sel_m1 & s_awready;

  assign m0_bvalid = s_bvalid & ~wr_sel_m1;
  assign m1_bvalid = s_bvalid &  wr_sel_m1;
  assign s_bready  = wr_sel_m1 ? m1_bready : m0_bready;

  logic wr_fire_aw, wr_fire_b;
  assign wr_fire_aw = s_awvalid & s_awready;
  assign wr_fire_b  = s_bvalid & s_bready;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      wr_pin_v <= 1'b0;
      wr_pin   <= 1'b0;
      rr_wr    <= 1'b0;
    end else begin
      if (wr_fire_aw) begin
        wr_pin_v <= 1'b1;
        wr_pin   <= wr_sel_m1;
        rr_wr    <= ~wr_sel_m1;
      end
      if (wr_fire_b) wr_pin_v <= 1'b0;
    end
  end

endmodule
