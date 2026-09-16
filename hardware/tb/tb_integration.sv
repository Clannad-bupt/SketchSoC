// Integration testbench for sketchsoc_top: exercises the integrated
// accelerator THROUGH ITS EXTERNAL PORTS ONLY (AXI4-Lite register port,
// packet ingress stream, L1 AXI master).  Unlike tb_ctrl_plane (which
// wires the nine submodules into a harness and peeks at internals), this
// bench instantiates the shipped top and validates end-to-end behavior:
//
//   IT1  post-reset STATUS/POOL reads
//   IT2  pool seeding via POOL_PUSH + live count readback
//   IT3  query install (2-block L1 CMS), DIR readback, LIFO pop order
//   IT4  100 packets -> telemetry snapshot: att=100, rej=0
//   IT5  sketch readout via RO_CMD vs the behavioural L1 memory (only
//        lines the DUT actually wrote are compared)
//   IT6  query remove: pools restored, directory invalid, att flat
//   IT7  scan re-enable + hot traffic -> heat-driven promote observed
//        through telemetry, no errors, pipeline keeps draining

`timescale 1ns/1ps

module tb_integration;

  import sketchsoc_pkg::*;

  localparam int unsigned QBLK_AW  = 4;
  localparam int unsigned L0_AW    = 11;
  localparam int unsigned L1_LINES = 2048;

  // register map (mirrors ctrl_plane.sv)
  localparam logic [11:0] A_CTRL   = 12'h000, A_STATUS = 12'h004,
                          A_QSEL   = 12'h008, A_QD0    = 12'h00C,
                          A_QS0    = 12'h010, A_QS1    = 12'h014,
                          A_QS2    = 12'h018, A_QS3    = 12'h01C,
                          A_QDHI   = 12'h020, A_QCFG   = 12'h024,
                          A_QCMD   = 12'h028, A_POOL   = 12'h02C,
                          A_DIRIDX = 12'h030, A_DENTLO = 12'h034,
                          A_DENTHI = 12'h038, A_ROCMD  = 12'h03C,
                          A_ROLO   = 12'h040, A_ROHI   = 12'h044,
                          A_TLM    = 12'h048;

  logic clk = 1'b0;
  logic rst_n = 1'b0;
  always #5 clk = ~clk;

  int errors = 0;

  // ------------------------------------------------------------------
  // DUT-facing signals
  // ------------------------------------------------------------------
  logic        m_awvalid, m_wvalid, m_arvalid;
  logic [11:0] m_awaddr, m_araddr;
  logic [31:0] m_wdata;
  logic        s_axi_awready, s_axi_wready, s_axi_bvalid;
  logic [1:0]  s_axi_bresp;
  logic        s_axi_arready, s_axi_rvalid;
  logic [31:0] s_axi_rdata;
  logic [1:0]  s_axi_rresp;

  logic      ing_val, ing_rdy;
  pkt_desc_t ing_pkt;

  logic       wo_val, wo_is_token;
  logic [3:0] wo_slot;

  // L1 backing store (DUT is the master)
  logic        x_arvalid, x_arready, x_rvalid, x_rready;
  logic [31:0] x_araddr;
  logic [63:0] x_rdata;
  logic        x_awvalid, x_awready, x_bvalid, x_bready;
  logic [31:0] x_awaddr;
  logic [63:0] x_awdata;
  logic [7:0]  x_awstrb;

  // ------------------------------------------------------------------
  // DUT
  // ------------------------------------------------------------------
  sketchsoc_top #(.QBLK_AW(QBLK_AW), .L0_AW(L0_AW),
                  .PROM_TH(512), .DEM_TH(32)) u_dut (
    .clk(clk), .rst_n(rst_n),
    .s_axi_awaddr(m_awaddr), .s_axi_awprot(3'd0), .s_axi_awvalid(m_awvalid),
    .s_axi_awready(s_axi_awready),
    .s_axi_wdata(m_wdata), .s_axi_wstrb(4'hF), .s_axi_wvalid(m_wvalid),
    .s_axi_wready(s_axi_wready),
    .s_axi_bresp(s_axi_bresp), .s_axi_bvalid(s_axi_bvalid),
    .s_axi_bready(1'b1),
    .s_axi_araddr(m_araddr), .s_axi_arprot(3'd0), .s_axi_arvalid(m_arvalid),
    .s_axi_arready(s_axi_arready),
    .s_axi_rdata(s_axi_rdata), .s_axi_rresp(s_axi_rresp),
    .s_axi_rvalid(s_axi_rvalid), .s_axi_rready(1'b1),
    .ing_val(ing_val), .ing_rdy(ing_rdy), .ing_pkt(ing_pkt),
    .wo_val(wo_val), .wo_slot(wo_slot), .wo_is_token(wo_is_token),
    .m_axi_arvalid(x_arvalid), .m_axi_arready(x_arready),
    .m_axi_araddr(x_araddr),
    .m_axi_rvalid(x_rvalid), .m_axi_rready(x_rready), .m_axi_rdata(x_rdata),
    .m_axi_awvalid(x_awvalid), .m_axi_awready(x_awready),
    .m_axi_awaddr(x_awaddr), .m_axi_awdata(x_awdata),
    .m_axi_awstrb(x_awstrb),
    .m_axi_bvalid(x_bvalid), .m_axi_bready(x_bready)
  );

  // ------------------------------------------------------------------
  // behavioural L1 (identical to the unit benches): 3-cycle read delay,
  // AW/W captured together, byte strobes, no RESP channel
  // ------------------------------------------------------------------
  logic [63:0] l1_mem [0:L1_LINES-1];
  initial for (int i = 0; i < L1_LINES; i++) l1_mem[i] = 64'h0;

  // track lines the DUT has written (IT5 compares only real content)
  bit wr_seen [int];

  logic [31:0] ar_addr_q, wr_addr_q;
  logic [63:0] wr_data_q;
  logic [7:0]  wr_strb_q;
  int unsigned rd_dly;
  logic        rd_pend, wr_pend;

  always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x_rvalid <= 1'b0; rd_pend <= 1'b0; rd_dly <= 0;
      x_bvalid <= 1'b0; wr_pend <= 1'b0;
    end else begin
      if (x_arvalid) begin
        rd_pend   <= 1'b1;
        rd_dly    <= 3;
        ar_addr_q <= x_araddr;
      end
      if (rd_pend && (rd_dly == 0)) begin
        if (x_rvalid && x_rready) begin
          x_rvalid <= 1'b0;
          rd_pend  <= 1'b0;
        end else begin
          x_rvalid <= 1'b1;
          x_rdata  <= l1_mem[ar_addr_q[31:3] % L1_LINES];
        end
      end else if (rd_pend) begin
        rd_dly <= rd_dly - 1;
      end
      if (x_awvalid) begin
        wr_pend   <= 1'b1;
        wr_addr_q <= x_awaddr;
        wr_data_q <= x_awdata;
        wr_strb_q <= x_awstrb;
      end
      if (wr_pend) begin
        for (int b = 0; b < 8; b++)
          if (wr_strb_q[b])
            l1_mem[wr_addr_q[31:3] % L1_LINES][8*b +: 8] <= wr_data_q[8*b +: 8];
        wr_seen[wr_addr_q[31:3] % L1_LINES] = 1;
        x_bvalid <= 1'b1;
        wr_pend  <= 1'b0;
      end
      if (x_bvalid && x_bready)
        x_bvalid <= 1'b0;
    end
  end

  assign x_arready = 1'b1;
  assign x_awready = 1'b1;

  // ------------------------------------------------------------------
  // AXI4-Lite master (negedge-aligned; b/r are 1-cycle pulses -- proven
  // pattern from tb_ctrl_plane, bounded waits everywhere)
  // ------------------------------------------------------------------
  task automatic axi_wr(input logic [11:0] addr, input logic [31:0] data);
    int to;
    begin
      while (s_axi_bvalid) @(negedge clk);
      m_awaddr = addr; m_wdata = data;
      m_awvalid = 1'b1; m_wvalid = 1'b1;
      @(negedge clk); @(negedge clk);
      m_awvalid = 1'b0; m_wvalid = 1'b0;
      to = 0;
      while (!s_axi_bvalid) begin
        @(negedge clk); to++;
        if (to > 100000) begin
          errors++;
          $display("FAIL axi_wr: bvalid timeout wr %h", addr);
          $finish;
        end
      end
      @(negedge clk);
    end
  endtask

  task automatic axi_rd(input logic [11:0] addr, output logic [31:0] data);
    int to;
    begin
      while (s_axi_rvalid) @(negedge clk);
      m_araddr  = addr; m_arvalid = 1'b1;
      to = 0;
      while (!s_axi_arready) begin
        @(negedge clk); to++;
        if (to > 100000) begin
          errors++; $display("FAIL axi_rd: arready timeout rd %h", addr);
          $finish;
        end
      end
      @(negedge clk);
      m_arvalid = 1'b0;
      to = 0;
      while (!s_axi_rvalid) begin
        @(negedge clk); to++;
        if (to > 100000) begin
          errors++; $display("FAIL axi_rd: rvalid timeout rd %h", addr);
          $finish;
        end
      end
      data = s_axi_rdata;
      @(negedge clk);
    end
  endtask

  // ------------------------------------------------------------------
  // helpers
  // ------------------------------------------------------------------
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

  // one packet; waits for the work-item completion strobe
  task automatic send_pkt(input flow_key_t key, input int id);
    int to;
    begin
      @(negedge clk);
      ing_val        = 1'b1;
      ing_pkt.key    = key;
      ing_pkt.pkt_id = id;
      ing_pkt.tstamp = 0;
      ing_pkt.rsv    = '0;
      to = 0;
      while (!ing_rdy) begin
        @(negedge clk); to++;
        if (to > 100000) begin
          errors++; $display("FAIL send_pkt: ing_rdy stuck"); $finish;
        end
      end
      @(negedge clk);
      ing_val = 1'b0;
      to = 0;
      while (!(wo_val && !wo_is_token)) begin
        @(negedge clk); to++;
        if (to > 200000) begin
          errors++; $display("FAIL send_pkt: no wo for id %0d", id); $finish;
        end
      end
    end
  endtask

  task automatic install_query(input int q, input qdesc_t D,
                               input int nblocks, input logic [1:0] tier,
                               input bit expect_err);
    logic [31:0] st;
    int t;
    begin
      axi_wr(A_QSEL, 32'(q));
      axi_wr(A_QD0,  (32'(D.kind) << 24) | (32'(D.rows) << 12) |
                     (32'(D.cols_log2) << 5) | 32'(D.width));
      axi_wr(A_QS0, D.seed0);
      axi_wr(A_QS1, D.seed1);
      axi_wr(A_QS2, D.seed2);
      axi_wr(A_QS3, D.seed3);
      axi_wr(A_QDHI, (32'(D.gen) << 24) | (32'(D.cand_log2) << 14) |
                     (32'(D.blm_k) << 10));
      axi_wr(A_QCFG, ((tier == TIER_L1) ? 32'h100 : 32'h0) | 32'(nblocks));
      axi_wr(A_QCMD, 32'h1);            // commit (enable stays 0)
      axi_wr(A_QCMD, 32'h2);            // install
      t = 0;
      forever begin
        axi_rd(A_STATUS, st);
        if (st[1] || st[3]) break;
        if (t++ > 20000) begin
          errors++;
          $display("FAIL install q%0d: no install_done/op_err within poll", q);
          break;
        end
      end
      if (expect_err ? !st[3] : (st[3] || !st[1])) begin
        errors++;
        $display("FAIL install q%0d: done=%0b err=%0b (expect_err=%0b)",
                 q, st[1], st[3], expect_err);
      end
      axi_wr(A_STATUS, 32'hA);          // W1C install_done + op_err
    end
  endtask

  task automatic remove_query(input int q);
    logic [31:0] st;
    int t;
    begin
      axi_wr(A_QSEL, 32'(q));
      axi_wr(A_QCMD, 32'h4);            // remove
      t = 0;
      forever begin
        axi_rd(A_STATUS, st);
        if (st[2] || st[3]) break;
        if (t++ > 200000) begin
          errors++;
          $display("FAIL remove q%0d: no remove_done within poll", q);
          break;
        end
      end
      if (st[3]) begin
        errors++;
        $display("FAIL remove q%0d: op_err set", q);
      end
      axi_wr(A_STATUS, 32'hC);          // W1C remove_done + op_err
    end
  endtask

  task automatic read_dir(input int q, input int blk, output dirent_t e);
    logic [31:0] lo, hi;
    begin
      axi_wr(A_DIRIDX, 32'(q) | (32'(blk) << 3));
      axi_rd(A_DENTLO, lo);
      axi_rd(A_DENTHI, hi);
      e = '0;
      e.valid = hi[31];
      e.tier  = hi[30:29];
      e.owner = hi[28:27];
      e.gen   = hi[26:19];
      e.mslot = hi[18:15];
      e.addr  = {hi[14:0], lo[31:15]};
    end
  endtask

  // L0: addr = word address | L1: addr = byte address
  task automatic readout(input logic [1:0] tier, input logic [31:0] addr,
                         output logic [31:0] lo, output logic [31:0] hi);
    logic [31:0] cmdw;
    begin
      if (tier == TIER_L0) cmdw = 32'h1 | (addr << 5);
      else                 cmdw = 32'h11 | ((addr >> 3) << 5);
      axi_wr(A_ROCMD, cmdw);            // deferred ack: data valid at return
      axi_rd(A_ROLO, lo);
      axi_rd(A_ROHI, hi);
    end
  endtask

  task automatic tlm_snap;
    begin
      axi_wr(A_TLM, 32'h1);
    end
  endtask

  // pool readback: [12:0]=l0_free, [28:16]=l1_free
  task automatic pool_check(input int exp_l0, input int exp_l1,
                            input string tag);
    logic [31:0] v;
    begin
      axi_rd(A_POOL, v);
      if (v[12:0] != 13'(exp_l0) || v[28:16] != 13'(exp_l1)) begin
        errors++;
        $display("FAIL %s pools: l0 %0d l1 %0d exp %0d/%0d",
                 tag, v[12:0], v[28:16], exp_l0, exp_l1);
      end
    end
  endtask

  // ------------------------------------------------------------------
  // stimulus
  // ------------------------------------------------------------------
  qdesc_t D;
  dirent_t e;
  logic [31:0] st, lo, hi, v;
  int ln;

  initial begin
    ing_val = 0; ing_pkt = '0;
    m_awvalid = 0; m_wvalid = 0; m_arvalid = 0;
    m_awaddr = 0; m_araddr = 0; m_wdata = 0;
    repeat (4) @(negedge clk);
    rst_n = 1'b1;
    repeat (2) @(negedge clk);

    // ---------------- IT1: post-reset register sanity ----------------
    axi_rd(A_STATUS, st);
    if (st !== 32'h200) begin               // [9] exec_idle set, rest clear
      errors++; $display("FAIL IT1 STATUS: got %h exp 00000200", st);
    end
    pool_check(0, 0, "IT1");
    $display("IT1 reset reads done");

    // scan defaults ON after reset; run the deterministic phases with it
    // off (a hot single-block query would legitimately migrate mid-count,
    // splitting att across direct/divert/replay by design)
    axi_wr(A_CTRL, 32'h0);

    // ---------------- IT2: pool seeding ------------------------------
    for (int k = 0; k < 4; k++)
      axi_wr(A_POOL, (32'(TIER_L0) << 30) | (32'(16 + k) * 64));
    for (int k = 0; k < 4; k++)
      axi_wr(A_POOL, (32'(TIER_L1) << 30) | (32'(8 + k) * 256));
    pool_check(4, 4, "IT2");
    $display("IT2 pool seed done");

    // ---------------- IT3: install q0 --------------------------------
    D = '0;
    D.enable = 1'b1; D.kind = KIND_CMS; D.rows = 2; D.cols_log2 = 7;
    D.width = WID_8; D.seed0 = 32'hA1B2C3D4; D.seed1 = 32'hB2C3D4E5;
    D.seed2 = 32'hC3D4E5F6; D.seed3 = 32'hD4E5F607; D.gen = 8'd5;
    install_query(0, D, 2, TIER_L1, 1'b0);
    pool_check(4, 2, "IT3");
    // LIFO pops: blk0 <- 0xB00, blk1 <- 0xA00
    read_dir(0, 0, e);
    if (!e.valid || e.tier != TIER_L1 || e.addr != 32'hB00 || e.gen != 8'd5) begin
      errors++;
      $display("FAIL IT3 dir(0,0): v%b t%0d g%0d a%h", e.valid, e.tier, e.gen, e.addr);
    end
    read_dir(0, 1, e);
    if (!e.valid || e.tier != TIER_L1 || e.addr != 32'hA00 || e.gen != 8'd5) begin
      errors++;
      $display("FAIL IT3 dir(0,1): v%b t%0d g%0d a%h", e.valid, e.tier, e.gen, e.addr);
    end
    $display("IT3 install done");

    // ---------------- IT4: 100 packets + telemetry --------------------
    for (int i = 0; i < 100; i++) send_pkt(gen_key(i), i);
    axi_rd(A_STATUS, st);
    if (!st[9]) begin
      errors++; $display("FAIL IT4 exec not idle after drain: %h", st);
    end
    tlm_snap;
    axi_rd(12'h050, v);                       // TLM_Q[0] = q0 attempted
    if (v !== 32'd100) begin
      errors++; $display("FAIL IT4 att: got %0d exp 100", v);
    end
    axi_rd(12'h054, v);                       // TLM_Q[1] = q0 rejected
    if (v !== 32'd0) begin
      errors++; $display("FAIL IT4 rej: got %0d exp 0", v);
    end
    axi_rd(12'h058, v);                       // TLM_Q[2] = q0 direct
    if (v !== 32'd100) begin
      errors++; $display("FAIL IT4 dir: got %0d exp 100", v);
    end
    $display("IT4 traffic + telemetry done");

    // ---------------- IT5: readout through the top --------------------
    // every line the DUT wrote must read back identical via RO_CMD
    if (wr_seen.num() == 0) begin
      errors++; $display("FAIL IT5: DUT never wrote L1");
    end else begin
      foreach (wr_seen[ln]) begin
        readout(TIER_L1, 32'(ln) * 8, lo, hi);
        if ({hi, lo} !== l1_mem[ln]) begin
          errors++;
          $display("FAIL IT5 L1 line %0d: got %h%h exp %h",
                   ln, hi, lo, l1_mem[ln]);
        end
      end
    end
    $display("IT5 readout done (%0d lines)", wr_seen.num());

    // ---------------- IT6: remove q0 ----------------------------------
    remove_query(0);
    pool_check(4, 4, "IT6");
    read_dir(0, 0, e);
    if (e.valid) begin
      errors++; $display("FAIL IT6 dir(0,0) still valid after remove");
    end
    read_dir(0, 1, e);
    if (e.valid) begin
      errors++; $display("FAIL IT6 dir(0,1) still valid after remove");
    end
    // removed query is silent: packets are skipped, att stays at 100
    for (int i = 100; i < 120; i++) send_pkt(gen_key(i), i);
    tlm_snap;
    axi_rd(12'h050, v);
    if (v !== 32'd100) begin
      errors++; $display("FAIL IT6 att moved after remove: got %0d exp 100", v);
    end
    $display("IT6 remove done");

    // ---------------- IT7: scan on -> heat-driven promote -------------
    axi_wr(A_CTRL, 32'h1);                    // scan_en = 1
    install_query(0, D, 2, TIER_L1, 1'b0);
    pool_check(4, 2, "IT7");
    begin
      bit promoted;
      promoted = 1'b0;
      for (int r = 0; r < 25 && !promoted; r++) begin
        for (int i = 0; i < 200; i++)
          send_pkt(gen_key(i % 8), 1000 + r * 200 + i);
        tlm_snap;
        axi_rd(12'h0C4, v);                   // TLM_G prom_cnt
        if (v > 0) promoted = 1'b1;
      end
      if (!promoted) begin
        errors++; $display("FAIL IT7: no promote within 5000 hot packets");
      end
    end
    // pipeline still drains after the migration
    for (int i = 0; i < 50; i++) send_pkt(gen_key(200 + i), 2000 + i);
    tlm_snap;
    axi_rd(12'h0C0, v);                       // TLM_G err_cnt
    if (v !== 32'd0) begin
      errors++; $display("FAIL IT7 err_cnt: got %0d exp 0", v);
    end
    // migration replay can still be draining right after the last packet
    // (HASH_WAIT added per-row hash latency): poll until exec idle and no
    // migration busy, with a bounded timeout
    begin
      int t;
      bit idle;
      idle = 1'b0;
      for (t = 0; t < 20000 && !idle; t++) begin
        axi_rd(A_STATUS, st);
        idle = st[9] && !st[8];
      end
      if (!idle) begin
        errors++; $display("FAIL IT7 exec not idle at end: %h", st);
      end
    end
    $display("IT7 scan + promote done");

    // ---------------- verdict -----------------------------------------
    if (errors == 0) $display("[tb_integration] ALL TESTS PASSED");
    else             $display("[tb_integration] %0d ERRORS", errors);
    $finish;
  end

  initial begin
    #100_000_000;
    $display("[tb_integration] TIMEOUT");
    $finish;
  end

endmodule
