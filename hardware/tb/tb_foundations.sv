// Foundations regression: l0_store / state_directory / heat_table.
//
// Checks:
//   L0        : 32-bit writes, byte-strobe sub-word cell writes (the
//               width-mixed 8/16/32-bit ALU's store side), both requestors,
//               round-robin fairness under simultaneous requests.
//   directory : entry round-trip through both write requestors and all
//               three read requestors (owner/tier/gen/mslot/addr fields).
//   heat      : EWMA convergence against a software model, decay halving,
//               non-destructive probes.

`timescale 1ns/1ps

module tb_foundations;

  import sketchsoc_pkg::*;

  localparam int unsigned L0_AW    = 10;   // 1024 words = 16 blocks
  localparam int unsigned QBLK_AW  = 4;    // small directory for the TB

  localparam int unsigned DIR_IDXW = QID_W + QBLK_AW;
  localparam int unsigned HEAT_IDXW = QID_W + QBLK_AW;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  int errors = 0;

  // ------------------------------------------------------------------
  // l0_store
  // ------------------------------------------------------------------
  logic              l0_r0_val, l0_r1_val, l0_w0_val, l0_w1_val;
  logic [L0_AW-1:0]  l0_r0_addr, l0_r1_addr, l0_w0_addr, l0_w1_addr;
  logic [31:0]       l0_r0_data, l0_r1_data, l0_w0_data, l0_w1_data;
  logic [3:0]        l0_w0_strb, l0_w1_strb;
  logic              l0_r0_rdy, l0_r1_rdy, l0_w0_rdy, l0_w1_rdy;
  logic              l0_r0_dv,  l0_r1_dv;

  l0_store #(.AW(L0_AW)) u_l0 (
    .clk(clk), .rst_n(rst_n),
    .r0_val(l0_r0_val), .r0_rdy(l0_r0_rdy), .r0_addr(l0_r0_addr),
    .r0_data(l0_r0_data), .r0_dv(l0_r0_dv),
    .r1_val(l0_r1_val), .r1_rdy(l0_r1_rdy), .r1_addr(l0_r1_addr),
    .r1_data(l0_r1_data), .r1_dv(l0_r1_dv),
    .w0_val(l0_w0_val), .w0_rdy(l0_w0_rdy), .w0_addr(l0_w0_addr),
    .w0_data(l0_w0_data), .w0_strb(l0_w0_strb),
    .w1_val(l0_w1_val), .w1_rdy(l0_w1_rdy), .w1_addr(l0_w1_addr),
    .w1_data(l0_w1_data), .w1_strb(l0_w1_strb)
  );

  // ------------------------------------------------------------------
  // state_directory
  // ------------------------------------------------------------------
  logic                  d_r0_val, d_r1_val, d_r2_val, d_w0_val, d_w1_val;
  logic [DIR_IDXW-1:0]   d_r0_idx, d_r1_idx, d_r2_idx, d_w0_idx, d_w1_idx;
  dirent_t               d_r0_ent, d_r1_ent, d_r2_ent, d_w0_ent, d_w1_ent;
  logic                  d_r0_rdy, d_r1_rdy, d_r2_rdy, d_w0_rdy, d_w1_rdy;
  logic                  d_r0_dv,  d_r1_dv,  d_r2_dv;

  state_directory #(.QBLK_AW(QBLK_AW)) u_dir (
    .clk(clk), .rst_n(rst_n),
    .r0_val(d_r0_val), .r0_rdy(d_r0_rdy), .r0_idx(d_r0_idx),
    .r0_ent(d_r0_ent), .r0_dv(d_r0_dv),
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

  // ------------------------------------------------------------------
  // heat_table
  // ------------------------------------------------------------------
  logic                    h_acc_val, h_scn_val, h_scn_decay;
  logic [HEAT_IDXW-1:0]    h_acc_idx, h_scn_idx;
  logic                    h_acc_rdy, h_scn_rdy, h_scn_dv;
  logic [15:0]             h_scn_heat;

  heat_table #(.QBLK_AW(QBLK_AW)) u_heat (
    .clk(clk), .rst_n(rst_n),
    .acc_val(h_acc_val), .acc_rdy(h_acc_rdy), .acc_idx(h_acc_idx),
    .scn_val(h_scn_val), .scn_rdy(h_scn_rdy), .scn_idx(h_scn_idx),
    .scn_decay(h_scn_decay), .scn_heat(h_scn_heat), .scn_dv(h_scn_dv)
  );

  // ------------------------------------------------------------------
  // helper tasks (drive at negedge, handshake at posedge)
  // ------------------------------------------------------------------
  task automatic l0_write(input int unsigned req,
                          input logic [L0_AW-1:0] a,
                          input logic [31:0] d,
                          input logic [3:0] s);
    begin
      @(negedge clk);
      if (req == 0) begin
        l0_w0_val = 1'b1; l0_w0_addr = a; l0_w0_data = d; l0_w0_strb = s;
        while (!l0_w0_rdy) @(negedge clk);
        @(negedge clk);
        l0_w0_val = 1'b0;
      end else begin
        l0_w1_val = 1'b1; l0_w1_addr = a; l0_w1_data = d; l0_w1_strb = s;
        while (!l0_w1_rdy) @(negedge clk);
        @(negedge clk);
        l0_w1_val = 1'b0;
      end
    end
  endtask

  task automatic l0_read(input int unsigned req,
                         input logic [L0_AW-1:0] a,
                         output logic [31:0] d);
    begin
      if (req == 0) begin
        @(negedge clk);
        l0_r0_val = 1'b1; l0_r0_addr = a;
        while (!l0_r0_dv) @(negedge clk);
        d = l0_r0_data;
        l0_r0_val = 1'b0;
      end else begin
        @(negedge clk);
        l0_r1_val = 1'b1; l0_r1_addr = a;
        while (!l0_r1_dv) @(negedge clk);
        d = l0_r1_data;
        l0_r1_val = 1'b0;
      end
    end
  endtask

  task automatic dir_write(input int unsigned req,
                           input logic [DIR_IDXW-1:0] idx,
                           input dirent_t e);
    begin
      @(negedge clk);
      if (req == 0) begin
        d_w0_val = 1'b1; d_w0_idx = idx; d_w0_ent = e;
        while (!d_w0_rdy) @(negedge clk);
        @(negedge clk);
        d_w0_val = 1'b0;
      end else begin
        d_w1_val = 1'b1; d_w1_idx = idx; d_w1_ent = e;
        while (!d_w1_rdy) @(negedge clk);
        @(negedge clk);
        d_w1_val = 1'b0;
      end
    end
  endtask

  task automatic dir_read(input int unsigned req,
                          input logic [DIR_IDXW-1:0] idx,
                          output dirent_t e);
    begin
      case (req)
        0: begin
          @(negedge clk);
          d_r0_val = 1'b1; d_r0_idx = idx;
          while (!d_r0_dv) @(negedge clk);
          e = d_r0_ent;
          d_r0_val = 1'b0;
        end
        1: begin
          @(negedge clk);
          d_r1_val = 1'b1; d_r1_idx = idx;
          while (!d_r1_dv) @(negedge clk);
          e = d_r1_ent;
          d_r1_val = 1'b0;
        end
        default: begin
          @(negedge clk);
          d_r2_val = 1'b1; d_r2_idx = idx;
          while (!d_r2_dv) @(negedge clk);
          e = d_r2_ent;
          d_r2_val = 1'b0;
        end
      endcase
    end
  endtask

  task automatic heat_access(input logic [HEAT_IDXW-1:0] idx);
    begin
      @(negedge clk);
      h_acc_val = 1'b1; h_acc_idx = idx;
      while (!h_acc_rdy) @(negedge clk);
      @(negedge clk);
      h_acc_val = 1'b0;
    end
  endtask

  task automatic heat_probe(input logic [HEAT_IDXW-1:0] idx,
                            input logic decay,
                            output logic [15:0] h);
    begin
      @(negedge clk);
      h_scn_val = 1'b1; h_scn_idx = idx; h_scn_decay = decay;
      while (!h_scn_dv) @(negedge clk);
      h = h_scn_heat;
      h_scn_val = 1'b0;
    end
  endtask

  function automatic logic [DIR_IDXW-1:0] mkidx(input logic [2:0] q,
                                                input logic [15:0] b);
    return {q, b[QBLK_AW-1:0]};
  endfunction

  // ------------------------------------------------------------------
  // stimulus
  // ------------------------------------------------------------------
  logic [31:0]   rd32;
  dirent_t       de;
  logic [15:0]   heat;
  logic [15:0]   hexp;
  int unsigned   i;

  initial begin
    l0_r0_val = 0; l0_r1_val = 0; l0_w0_val = 0; l0_w1_val = 0;
    l0_r0_addr = 0; l0_r1_addr = 0; l0_w0_addr = 0; l0_w1_addr = 0;
    l0_w0_data = 0; l0_w1_data = 0; l0_w0_strb = 0; l0_w1_strb = 0;
    d_r0_val = 0; d_r1_val = 0; d_r2_val = 0; d_w0_val = 0; d_w1_val = 0;
    d_r0_idx = 0; d_r1_idx = 0; d_r2_idx = 0; d_w0_idx = 0; d_w1_idx = 0;
    d_w0_ent = '0; d_w1_ent = '0;
    h_acc_val = 0; h_scn_val = 0; h_scn_decay = 0;
    h_acc_idx = 0; h_scn_idx = 0;

    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ---- L0: full-word write, read back through both requestors -------
    l0_write(0, 10'd5, 32'hAABBCCDD, 4'b1111);
    l0_read(0, 10'd5, rd32);
    if (rd32 !== 32'hAABBCCDD) begin
      errors++;
      $display("FAIL L0 r0 full-word: got %h exp AABBCCDD", rd32);
    end
    l0_read(1, 10'd5, rd32);
    if (rd32 !== 32'hAABBCCDD) begin
      errors++;
      $display("FAIL L0 r1 full-word: got %h exp AABBCCDD", rd32);
    end

    // ---- L0: byte-strobe sub-word writes (8-bit cell at offset 1, ------
    // ----       16-bit cell at offset 2 via the other requestor) --------
    l0_write(0, 10'd6, 32'h11223344, 4'b1111);
    l0_write(0, 10'd6, 32'h0000EE00, 4'b0010);
    l0_write(1, 10'd6, 32'hFF990000, 4'b1100);
    l0_read(1, 10'd6, rd32);
    if (rd32 !== 32'hFF99EE44) begin
      errors++;
      $display("FAIL L0 byte-strobe: got %h exp FF99EE44", rd32);
    end

    // ---- L0: simultaneous requests on both read requestors -------------
    fork
      l0_read(0, 10'd5, rd32);
      l0_read(1, 10'd6, rd32);
    join
    // rd32 holds the later result of the two; re-read serially to check
    l0_read(0, 10'd5, rd32);
    if (rd32 !== 32'hAABBCCDD) begin errors++; $display("FAIL L0 rr r0"); end
    l0_read(1, 10'd6, rd32);
    if (rd32 !== 32'hFF99EE44) begin errors++; $display("FAIL L0 rr r1"); end

    // ---- directory: entry round-trip ------------------------------------
    d_w1_ent = '0;
    d_w1_ent.valid = 1'b1;
    d_w1_ent.tier  = TIER_L1;
    d_w1_ent.owner = OWN_MIGRATING;
    d_w1_ent.gen   = 8'd42;
    d_w1_ent.mslot = 4'd3;
    d_w1_ent.addr  = 32'h0000BEE0;
    dir_write(1, mkidx(3'd2, 16'd7), d_w1_ent);

    dir_read(0, mkidx(3'd2, 16'd7), de);
    if (!(de.valid && de.tier == TIER_L1 && de.owner == OWN_MIGRATING &&
          de.gen == 8'd42 && de.mslot == 4'd3 && de.addr == 32'h0000BEE0)) begin
      errors++;
      $display("FAIL dir r0 round-trip: v=%b t=%0d o=%0d g=%0d s=%0d a=%h",
               de.valid, de.tier, de.owner, de.gen, de.mslot, de.addr);
    end
    dir_read(1, mkidx(3'd2, 16'd7), de);
    if (de.addr !== 32'h0000BEE0) begin errors++; $display("FAIL dir r1"); end
    dir_read(2, mkidx(3'd2, 16'd7), de);
    if (de.addr !== 32'h0000BEE0) begin errors++; $display("FAIL dir r2"); end

    // engine-side write (freeze CAS result) via w0
    d_w0_ent = de;
    d_w0_ent.owner = OWN_REPLAYING;
    d_w0_ent.mslot = 4'd5;
    dir_write(0, mkidx(3'd2, 16'd7), d_w0_ent);
    dir_read(2, mkidx(3'd2, 16'd7), de);
    if (!(de.owner == OWN_REPLAYING && de.mslot == 4'd5 && de.gen == 8'd42)) begin
      errors++; $display("FAIL dir w0 update: o=%0d s=%0d", de.owner, de.mslot);
    end

    // ---- heat: EWMA model match -----------------------------------------
    hexp = 16'd0;
    for (i = 0; i < 60; i++) begin
      heat_access(mkidx(3'd1, 16'd9));
      hexp = hexp + 16'd512 - (hexp >> 3);
    end
    heat_probe(mkidx(3'd1, 16'd9), 1'b0, heat);
    if (heat !== hexp) begin
      errors++;
      $display("FAIL heat EWMA: got %0d exp %0d", heat, hexp);
    end
    // non-destructive probe
    heat_probe(mkidx(3'd1, 16'd9), 1'b0, heat);
    if (heat !== hexp) begin errors++; $display("FAIL heat probe nd"); end
    // decay: three halvings
    for (i = 0; i < 3; i++) begin
      heat_probe(mkidx(3'd1, 16'd9), 1'b1, heat);
      hexp = hexp >> 1;
      if (heat !== hexp) begin
        errors++;
        $display("FAIL heat decay %0d: got %0d exp %0d", i, heat, hexp);
      end
    end
    // untouched index stays zero
    heat_probe(mkidx(3'd1, 16'd10), 1'b0, heat);
    if (heat !== 16'd0) begin errors++; $display("FAIL heat untouched"); end

    repeat (4) @(negedge clk);
    if (errors == 0)
      $display("[tb_foundations] ALL TESTS PASSED");
    else
      $display("[tb_foundations] %0d ERRORS", errors);
    $finish;
  end

  // safety net
  initial begin
    #100_000;
    $display("[tb_foundations] TIMEOUT");
    $finish;
  end

endmodule
