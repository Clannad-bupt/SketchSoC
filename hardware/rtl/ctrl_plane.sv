// Control plane (paper Sec. 3.5): query lifecycle over an AXI4-Lite slave.
//
// The data-path modules take no configuration side effects from software;
// this block owns all of them:
//
//   * query descriptors   -- staged via QSEL/QDESC_* registers, committed
//     atomically (enable held 0 until the install FSM publishes it last:
//     the paper's Prepare -> Reserve -> Publish ordering).
//   * install             -- pop physical blocks from the tier manager's
//     free stacks, write directory entries, then publish enable=1.  On
//     quota exhaustion mid-way it rolls back (entries invalidated, popped
//     addresses pushed back LIFO) and reports op_err.
//   * remove              -- quiesce (enable=0, gen++ so stale replay
//     tokens fail the epoch guard, packet source back-pressured through
//     quiesce_req_o), wait for engine+exec idle, then walk every block of
//     the query: invalidate the entry and push the physical block back.
//   * sketch readout      -- L0 word / L1 64-bit beat read through the
//     migration engine's idle memory ports (bypass muxes, no changes to
//     l0_store / l1_arb2).  cmd_hold blocks new migration commands; the
//     engine is sampled idle one state after cmd_hold rises, which is
//     airtight: the last possible command handshake happens on the edge
//     cmd_hold asserts, making busy high in the very cycle sampled.
//   * telemetry snapshot  -- 24 SMU counters (combinational acc_rd, one
//     cycle per slot) + 10 globals latched for readback.
//
// Interface constraints honoured here:
//   * every free-stack push is deferred to a cycle with rel_val == 0 (the
//     tier manager drops push_addr when a release lands the same cycle);
//   * install/remove never use cmd_hold -- the tier manager parked in
//     S_CMD would deadlock the pop port (pop is gated on the scan state);
//     they wait for !mig_busy && !cmd_val instead;
//   * AW and W are latched independently (no same-cycle assumption);
//     responses stay in order (a write executes only when no deferred
//     response is outstanding);
//   * the scan is disabled during install/remove/readout FSMs (scn_en_o),
//     but not during telemetry snapshots.
//
// v1 limitations (documented in the delivery notes): install onto a query
// that is already enabled is rejected (op_err) -- remove first; reinstall
// inherits the query's old heat (no per-query heat clear); remove discards
// in-flight updates (paper quiesce semantics); HH candidate readout is not
// wired in this phase.

`timescale 1ns/1ps

module ctrl_plane import sketchsoc_pkg::*; #(
  parameter int unsigned QBLK_AW = 12,
  parameter int unsigned L0_AW   = 12
)(
  input  logic clk,
  input  logic rst_n,

  // ---- AXI4-Lite slave (12-bit byte address) ------------------------------
  input  logic [11:0] s_axi_awaddr,
  input  logic [2:0]  s_axi_awprot,
  input  logic        s_axi_awvalid,
  output logic        s_axi_awready,
  input  logic [31:0] s_axi_wdata,
  input  logic [3:0]  s_axi_wstrb,
  input  logic        s_axi_wvalid,
  output logic        s_axi_wready,
  output logic [1:0]  s_axi_bresp,
  output logic        s_axi_bvalid,
  input  logic        s_axi_bready,
  input  logic [11:0] s_axi_araddr,
  input  logic [2:0]  s_axi_arprot,
  input  logic        s_axi_arvalid,
  output logic        s_axi_arready,
  output logic [31:0] s_axi_rdata,
  output logic [1:0]  s_axi_rresp,
  output logic        s_axi_rvalid,
  input  logic        s_axi_rready,

  // ---- query descriptors (dynamic configuration state) --------------------
  output qdesc_t      qd_o [QNUM],
  output logic [7:0]  qgen_o [QNUM],

  // ---- directory: write requestor 1 (exclusive), read requestor 3 ---------
  output logic                    dw_val,
  input  logic                    dw_rdy,
  output logic [QID_W+QBLK_AW-1:0] dw_idx,
  output dirent_t                 dw_ent,
  output logic                    dr_val,
  input  logic                    dr_rdy,
  output logic [QID_W+QBLK_AW-1:0] dr_idx,
  input  dirent_t                 dr_ent,
  input  logic                    dr_dv,

  // ---- tier-manager pool interface -----------------------------------------
  output logic        push_val,
  output logic [1:0]  push_tier,
  output logic [31:0] push_addr,
  output logic        flush_stacks,
  output logic        scn_en_o,
  output logic        pop_val,
  output logic [1:0]  pop_tier,
  input  logic        pop_rdy,
  input  logic [31:0] pop_addr,
  input  logic        pop_empty,
  input  logic        rel_val,       // monitored: pushes avoid release cycles
  input  logic        tm_cmd_val,    // raw tier-manager cmd_val (quiesce wait)

  // ---- migration engine gate / status ---------------------------------------
  output logic        cmd_hold,      // gates tier_mgr cmd_val at the top level
  output logic        quiesce_req_o, // packet-source backpressure (remove)
  input  logic        mig_busy,
  input  logic        exec_idle,

  // ---- L0 r1 bypass (engine <-> ctrl -> l0_store r1) ------------------------
  input  logic             eng_l0r_val,
  output logic             eng_l0r_rdy,
  input  logic [L0_AW-1:0] eng_l0r_addr,
  output logic [31:0]      eng_l0r_data,
  output logic             eng_l0r_dv,
  output logic             st_l0r_val,
  input  logic             st_l0r_rdy,
  output logic [L0_AW-1:0] st_l0r_addr,
  input  logic [31:0]      st_l0r_data,
  input  logic             st_l0r_dv,

  // ---- L1 m1 bypass (engine <-> ctrl -> l1_arb2 m1) --------------------------
  input  logic        eng_l1_arvalid,
  output logic        eng_l1_arready,
  input  logic [31:0] eng_l1_araddr,
  output logic        eng_l1_rvalid,
  input  logic        eng_l1_rready,
  output logic [63:0] eng_l1_rdata,
  input  logic        eng_l1_awvalid,
  output logic        eng_l1_awready,
  input  logic [31:0] eng_l1_awaddr,
  input  logic [63:0] eng_l1_awdata,
  input  logic [7:0]  eng_l1_awstrb,
  output logic        eng_l1_bvalid,
  input  logic        eng_l1_bready,
  output logic        st_l1_arvalid,
  input  logic        st_l1_arready,
  output logic [31:0] st_l1_araddr,
  input  logic        st_l1_rvalid,
  output logic        st_l1_rready,
  input  logic [63:0] st_l1_rdata,
  output logic        st_l1_awvalid,
  input  logic        st_l1_awready,
  output logic [31:0] st_l1_awaddr,
  output logic [63:0] st_l1_awdata,
  output logic [7:0]  st_l1_awstrb,
  input  logic        st_l1_bvalid,
  output logic        st_l1_bready,

  // ---- telemetry inputs -------------------------------------------------------
  input  logic [31:0] mig_cnt, rep_ok_cnt, rep_drop_cnt, cmd_rej_cnt, err_cnt,
  input  logic [31:0] prom_cnt, dem_cnt, scan_passes, l0_free_cnt, l1_free_cnt,
  output logic [4:0]  acc_sel,
  input  logic [31:0] acc_rd
);

  localparam int unsigned NBLKS = 1 << QBLK_AW;
  localparam logic [QBLK_AW-1:0] BLK_LAST = NBLKS - 1;   // last block index
  localparam logic [4:0]        MAX_NBLK = 5'd16;        // rollback array depth

  // ------------------------------------------------------------------
  // register map (byte addresses)
  // ------------------------------------------------------------------
  localparam logic [11:0] A_CTRL   = 12'h000,  // [0]=scan_en RW, [1]=flush W1
                          A_STATUS = 12'h004,  // [0]fsm_busy [1]inst_done W1C
                                               // [2]rm_done W1C [3]op_err W1C
                                               // [8]mig_busy [9]exec_idle
                          A_QSEL   = 12'h008,  // [2:0] qid
                          A_QD0    = 12'h00C,  // {kind,rows,cols_log2,width}
                          A_QS0    = 12'h010,
                          A_QS1    = 12'h014,
                          A_QS2    = 12'h018,
                          A_QS3    = 12'h01C,
                          A_QDHI   = 12'h020,  // {gen,cand_log2,blm_k}
                          A_QCFG   = 12'h024,  // {init_tier@8, nblocks@4:0}
                          A_QCMD   = 12'h028,  // [0]commit [1]install [2]remove
                          A_POOL   = 12'h02C,  // W {tier@31:30, addr@29:0}
                                               // R [12:0]l0_free [28:16]l1_free
                          A_DIRIDX = 12'h030,  // W {qid@2:0, blk@15:3} -> r3
                          A_DENTLO = 12'h034,  // R dirent bits [31:0]
                          A_DENTHI = 12'h038,  // R dirent bits [63:32]
                          A_ROCMD  = 12'h03C,  // W [0]start [4]tier
                                               //   [31:5]addr: L0 word addr |
                                               //   L1 line (byte addr >> 3)
                          A_ROLO   = 12'h040,  // R readout data [31:0]
                          A_ROHI   = 12'h044,  // R readout data [63:32] (L1)
                          A_TLM    = 12'h048;  // W [0] snapshot trigger
  // TLM_Q: 0x050 + 4*i, i = q*6+c  (24 registers)
  // TLM_G: 0x0B0 + 4*j, j = 0..9  (mig, rep_ok, rep_drop, cmd_rej, err,
  //                                prom, dem, scan_passes, l0_free, l1_free)

  localparam logic [2:0] DACK_NONE = 3'd0, DACK_POOL = 3'd1, DACK_DIR = 3'd2,
                         DACK_RO   = 3'd3, DACK_TLM  = 3'd4;

  // ------------------------------------------------------------------
  // register file
  // ------------------------------------------------------------------
  logic        reg_scan_en;
  logic [2:0]  qsel;
  logic [7:0]  qw_kind, qw_gen;
  logic [3:0]  qw_rows;
  logic [5:0]  qw_cols;
  logic [1:0]  qw_width;
  logic [31:0] qw_seed [0:3];
  logic [4:0]  qw_cand;
  logic [2:0]  qw_blmk;
  logic [4:0]  qcfg_nblk;
  logic        qcfg_tier;                 // 0 = L0, 1 = L1

  qdesc_t      qd_r [0:QNUM-1];

  logic        st_install_done, st_remove_done, st_op_err;

  dirent_t     dir_ent_r;                 // last directory readback
  logic [31:0] ro_lo_r, ro_hi_r;          // last sketch readout
  logic [31:0] tlm_q_r [0:23];
  logic [31:0] tlm_g_r [0:9];

  always_comb begin
    for (int q = 0; q < QNUM; q++) begin
      qd_o[q]   = qd_r[q];
      qgen_o[q] = qd_r[q].gen;
    end
  end

  // ------------------------------------------------------------------
  // pending-push unit: every stack push (AXI pool write, install rollback,
  // remove return) is serialised through one of two slots and fires only
  // on cycles with no engine release (rel+push the same cycle would drop
  // the push in the tier manager) and no flush pulse.  FSM pushes win.
  // ------------------------------------------------------------------
  logic        fpp_full;
  logic [1:0]  fpp_tier;
  logic [31:0] fpp_addr;
  logic        app_full;
  logic [1:0]  app_tier;
  logic [31:0] app_addr;
  logic        flush_r;

  wire pp_ok      = ~rel_val & ~flush_r;
  wire pp_fire_fsm = fpp_full & pp_ok;
  wire pp_fire_axi = app_full & pp_ok & ~pp_fire_fsm;

  assign push_val  = pp_fire_fsm | pp_fire_axi;
  assign push_tier = pp_fire_fsm ? fpp_tier : app_tier;
  assign push_addr = pp_fire_fsm ? fpp_addr : app_addr;
  assign flush_stacks = flush_r;

  // ------------------------------------------------------------------
  // main FSM states (declared early: the read mux below exposes fsm_busy)
  // ------------------------------------------------------------------
  typedef enum logic [4:0] {
    C_IDLE,
    C_INS_WAIT, C_INS_POP, C_INS_WR, C_INS_PUB, C_INS_RB, C_INS_RBP,
    C_RM_WAIT, C_RM_RD, C_RM_INV, C_RM_PSH, C_RM_ADV,
    C_DIR_RD,
    C_RO_HOLD, C_RO_CHK, C_RO_L0, C_RO_L1AR, C_RO_L1R, C_RO_REL,
    C_TLM
  } state_e;
  state_e state;

  // ------------------------------------------------------------------
  // AXI write channel: independent AW/W latching, single outstanding
  // transaction, in-order responses (execute only when no deferred ack).
  // ------------------------------------------------------------------
  logic        aw_v, w_v, b_v;
  logic [11:0] aw_addr_q;
  logic [31:0] w_data_q;

  assign s_axi_awready = ~aw_v & ~b_v;
  assign s_axi_wready  = ~w_v  & ~b_v;
  assign s_axi_bvalid  = b_v;
  assign s_axi_bresp   = 2'b00;

  // AXI read channel: address latched at ar accept (rdata stable while
  // rvalid -- araddr must not be sampled after the handshake), one
  // outstanding beat, no read side effects.
  logic        r_v;
  logic [11:0] ar_addr_q;
  logic [31:0] r_data_q;
  assign s_axi_arready = ~r_v;
  assign s_axi_rresp   = 2'b00;
  assign s_axi_rvalid  = r_v;
  assign s_axi_rdata   = r_data_q;

  always_comb begin
    case (ar_addr_q)
      A_CTRL:    r_data_q = {31'd0, reg_scan_en};
      A_STATUS:  r_data_q = {22'd0, exec_idle, mig_busy, 4'd0,
                             st_op_err, st_remove_done, st_install_done,
                             (state != C_IDLE)};
      A_QSEL:    r_data_q = {29'd0, qsel};
      A_POOL:    r_data_q = {3'd0, l1_free_cnt[12:0], 3'd0, l0_free_cnt[12:0]};
      A_DENTLO:  r_data_q = dir_ent_r[31:0];
      A_DENTHI:  r_data_q = dir_ent_r[63:32];
      A_ROLO:    r_data_q = ro_lo_r;
      A_ROHI:    r_data_q = ro_hi_r;
      default: begin
        if (ar_addr_q inside {[12'h050:12'h0AC]})       // TLM_Q q*6+c
          r_data_q = tlm_q_r[(ar_addr_q - 12'h050) >> 2];
        else if (ar_addr_q inside {[12'h0B0:12'h0D4]})  // TLM_G 0..9
          r_data_q = tlm_g_r[(ar_addr_q - 12'h0B0) >> 2];
        else
          r_data_q = 32'h0;
      end
    endcase
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      r_v <= 1'b0;
    end else begin
      if (s_axi_arvalid && s_axi_arready) begin
        r_v       <= 1'b1;
        ar_addr_q <= s_axi_araddr;
      end else if (r_v && s_axi_rready)
        r_v <= 1'b0;
    end
  end

  // ------------------------------------------------------------------
  // main FSM (also owns the register file -- single always_ff so no
  // variable ever has two drivers)
  // ------------------------------------------------------------------

  // request latches (set by write decode, consumed by the FSM)
  logic req_ins, req_rm, req_dir, req_ro, req_tlm;
  logic [2:0]       dir_req_q;
  logic [15:0]      dir_req_b;
  logic             ro_tier_r;
  logic [26:0]      ro_field_r;
  logic [2:0]       dack_kind;

  // op context
  logic [2:0]       op_q;
  logic [4:0]       op_nblk;
  logic             op_tier_l1;
  logic [QBLK_AW-1:0] op_b;
  int               rb_i, rb_j;
  logic [31:0]      rb_addr [0:15];
  logic [1:0]       rm_tier_r;
  logic [31:0]      rm_addr_r;

  // wait counters
  logic [3:0]  quiet_cnt, eq_cnt;
  logic [19:0] tot_cnt, ro_timeout;
  logic [4:0]  tlm_i;

  // held outputs of the readout FSM
  logic        cmd_hold_r, ro_hold_r;
  logic        ro_l0_val_r;
  logic [L0_AW-1:0] ro_l0_addr_r;
  logic        ro_l1_arval_r, ro_l1_rrdy_r;
  logic [31:0] ro_l1_addr_r;
  logic        blk_scan_r;

  assign cmd_hold    = cmd_hold_r;
  assign quiesce_req_o = (state == C_RM_WAIT) | (state == C_RM_RD) |
                         (state == C_RM_INV) | (state == C_RM_PSH) |
                         (state == C_RM_ADV);
  assign scn_en_o    = reg_scan_en & ~blk_scan_r;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= C_IDLE;
      aw_v           <= 1'b0;
      w_v            <= 1'b0;
      b_v            <= 1'b0;
      aw_addr_q      <= '0;
      w_data_q       <= '0;
      r_v            <= 1'b0;
      ar_addr_q      <= '0;
      dack_kind      <= DACK_NONE;
      req_ins        <= 1'b0;
      req_rm         <= 1'b0;
      req_dir        <= 1'b0;
      req_ro         <= 1'b0;
      req_tlm        <= 1'b0;
      dir_req_q      <= '0;
      dir_req_b      <= '0;
      ro_tier_r      <= 1'b0;
      ro_field_r     <= '0;
      reg_scan_en    <= 1'b1;
      qsel           <= '0;
      qw_kind        <= '0;  qw_gen <= '0;  qw_rows <= '0;
      qw_cols        <= '0;  qw_width <= '0; qw_cand <= '0;  qw_blmk <= '0;
      for (int i = 0; i < 4; i++) qw_seed[i] <= '0;
      qcfg_nblk      <= '0;
      qcfg_tier      <= 1'b0;
      for (int q = 0; q < QNUM; q++) qd_r[q] <= '0;
      st_install_done <= 1'b0;
      st_remove_done  <= 1'b0;
      st_op_err       <= 1'b0;
      fpp_full       <= 1'b0;  fpp_tier <= '0;  fpp_addr <= '0;
      app_full       <= 1'b0;  app_tier <= '0;  app_addr <= '0;
      flush_r        <= 1'b0;
      pop_val        <= 1'b0;
      pop_tier       <= '0;
      dr_val         <= 1'b0;
      dr_idx         <= '0;
      dw_val         <= 1'b0;
      dw_idx         <= '0;
      dw_ent         <= '0;
      cmd_hold_r     <= 1'b0;
      ro_hold_r      <= 1'b0;
      ro_l0_val_r    <= 1'b0;
      ro_l0_addr_r   <= '0;
      ro_l1_arval_r  <= 1'b0;
      ro_l1_rrdy_r   <= 1'b0;
      ro_l1_addr_r   <= '0;
      blk_scan_r     <= 1'b0;
      quiet_cnt      <= '0;
      eq_cnt         <= '0;
      tot_cnt        <= '0;
      ro_timeout     <= '0;
      tlm_i          <= '0;
      acc_sel        <= '0;
      op_q           <= '0;
      op_nblk        <= '0;
      op_tier_l1     <= 1'b0;
      op_b           <= '0;
      rb_i           <= 0;
      rb_j           <= 0;
      rm_tier_r      <= '0;
      rm_addr_r      <= '0;
      dir_ent_r      <= '0;
      ro_lo_r        <= '0;
      ro_hi_r        <= '0;
      for (int i = 0; i < 24; i++) tlm_q_r[i] <= '0;
      for (int i = 0; i < 10; i++) tlm_g_r[i] <= '0;
    end else begin
      // -------- defaults for single-cycle pulses ----------------------
      flush_r     <= 1'b0;
      pop_val     <= 1'b0;

      // -------- AW / W latching ---------------------------------------
      if (s_axi_awvalid && s_axi_awready) begin
        aw_v      <= 1'b1;
        aw_addr_q <= s_axi_awaddr;
      end
      if (s_axi_wvalid && s_axi_wready) begin
        w_v      <= 1'b1;
        w_data_q <= s_axi_wdata;
      end
      if (b_v && s_axi_bready)
        b_v <= 1'b0;

      // -------- pending-push fire / deferred ack -----------------------
      if (pp_fire_fsm) fpp_full <= 1'b0;
      if (pp_fire_axi) begin
        app_full <= 1'b0;
        if (dack_kind == DACK_POOL) begin
          b_v      <= 1'b1;
          dack_kind <= DACK_NONE;
        end
      end

      // -------- write decode (runs before the FSM so a same-edge FSM
      //          set of a sticky status bit wins over a W1C clear) -------
      if (aw_v && w_v && !b_v && (dack_kind == DACK_NONE)) begin
        case (aw_addr_q)
          A_CTRL: begin
            reg_scan_en <= w_data_q[0];
            if (w_data_q[1]) begin
              if ((state == C_IDLE) && !mig_busy &&
                  !fpp_full && !app_full)
                flush_r <= 1'b1;
              else
                st_op_err <= 1'b1;
            end
            aw_v <= 1'b0;  w_v <= 1'b0;  b_v <= 1'b1;
          end

          A_STATUS: begin
            if (w_data_q[1]) st_install_done <= 1'b0;
            if (w_data_q[2]) st_remove_done  <= 1'b0;
            if (w_data_q[3]) st_op_err       <= 1'b0;
            aw_v <= 1'b0;  w_v <= 1'b0;  b_v <= 1'b1;
          end

          A_QSEL:   begin qsel <= w_data_q[2:0];
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QD0:    begin qw_kind  <= w_data_q[31:24];
                          qw_rows  <= w_data_q[15:12];
                          qw_cols  <= w_data_q[10:5];
                          qw_width <= w_data_q[1:0];
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QS0:    begin qw_seed[0] <= w_data_q;
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QS1:    begin qw_seed[1] <= w_data_q;
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QS2:    begin qw_seed[2] <= w_data_q;
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QS3:    begin qw_seed[3] <= w_data_q;
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QDHI:   begin qw_gen  <= w_data_q[31:24];
                          qw_cand <= w_data_q[18:14];
                          qw_blmk <= w_data_q[12:10];
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end
          A_QCFG:   begin qcfg_nblk <= (w_data_q[4:0] > MAX_NBLK) ? MAX_NBLK
                                                                 : w_data_q[4:0];
                          qcfg_tier <= w_data_q[8];
                          aw_v <= 1'b0; w_v <= 1'b0; b_v <= 1'b1; end

          A_QCMD: begin
            if (w_data_q[0]) begin                  // commit descriptor
              // reject: live query, or this query's install is mid-FSM
              // (C_INS_PUB would publish a descriptor the blocks were
              // not installed for)
              if (qd_r[qsel].enable)
                st_op_err <= 1'b1;
              else if ((state == C_IDLE) || (op_q != qsel))
                qd_r[qsel] <= '{default: '0,
                                enable: 1'b0, kind: qw_kind, rows: qw_rows,
                                cols_log2: qw_cols, width: qw_width,
                                seed0: qw_seed[0], seed1: qw_seed[1],
                                seed2: qw_seed[2], seed3: qw_seed[3],
                                cand_log2: qw_cand, blm_k: qw_blmk,
                                gen: qw_gen};
            end
            if (w_data_q[1] && (state == C_IDLE) &&
                !req_ins && !req_rm && !req_dir && !req_ro && !req_tlm) begin
              if (qd_r[qsel].enable)
                st_op_err <= 1'b1;                  // install onto live query
              else begin
                req_ins    <= 1'b1;
                op_q       <= qsel;
                op_nblk    <= qcfg_nblk;
                op_tier_l1 <= qcfg_tier;
              end
            end
            if (w_data_q[2] && (state == C_IDLE) &&
                !req_ins && !req_rm && !req_dir && !req_ro && !req_tlm) begin
              req_rm <= 1'b1;
              op_q   <= qsel;
            end
            aw_v <= 1'b0;  w_v <= 1'b0;  b_v <= 1'b1;
          end

          A_POOL: begin
            if (!app_full) begin
              app_full  <= 1'b1;
              app_tier  <= w_data_q[31:30];
              app_addr  <= {2'd0, w_data_q[29:0]};
              dack_kind <= DACK_POOL;
              aw_v <= 1'b0;  w_v <= 1'b0;
            end
            // else: hold AW/W and retry next cycle
          end

          A_DIRIDX: begin
            if ((state == C_IDLE) && !req_dir && !req_ins && !req_rm &&
                !req_ro && !req_tlm) begin
              req_dir   <= 1'b1;
              dir_req_q <= w_data_q[2:0];
              dir_req_b <= w_data_q[15:3];
              dack_kind <= DACK_DIR;
              aw_v <= 1'b0;  w_v <= 1'b0;
            end
          end

          A_ROCMD: begin
            if (w_data_q[0]) begin
              if ((state == C_IDLE) && !req_ro && !req_ins && !req_rm &&
                  !req_dir && !req_tlm) begin
                req_ro     <= 1'b1;
                ro_tier_r  <= w_data_q[4];
                ro_field_r <= w_data_q[31:5];
                dack_kind  <= DACK_RO;
                aw_v <= 1'b0;  w_v <= 1'b0;
              end
            end else begin
              aw_v <= 1'b0;  w_v <= 1'b0;  b_v <= 1'b1;
            end
          end

          A_TLM: begin
            if (w_data_q[0]) begin
              if ((state == C_IDLE) && !req_tlm && !req_ins && !req_rm &&
                  !req_dir && !req_ro) begin
                req_tlm   <= 1'b1;
                dack_kind <= DACK_TLM;
                aw_v <= 1'b0;  w_v <= 1'b0;
              end
            end else begin
              aw_v <= 1'b0;  w_v <= 1'b0;  b_v <= 1'b1;
            end
          end

          default: begin                              // RO / unmapped: ack
            aw_v <= 1'b0;  w_v <= 1'b0;  b_v <= 1'b1;
          end
        endcase
      end

      // ------------------------- FSM ----------------------------------
      case (state)
        C_IDLE: begin
          if (req_ins) begin
            req_ins    <= 1'b0;
            blk_scan_r <= 1'b1;
            quiet_cnt  <= '0;
            tot_cnt    <= '0;
            op_b       <= '0;
            // pop_tier must be valid before the first C_INS_POP cycle:
            // pop_empty muxes on it
            pop_tier   <= op_tier_l1 ? TIER_L1 : TIER_L0;
            state      <= C_INS_WAIT;
          end else if (req_rm) begin
            req_rm        <= 1'b0;
            blk_scan_r    <= 1'b1;
            // epoch bump + disable first: in-flight smu work of this query
            // is what the quiesce wait below drains
            qd_r[op_q].enable <= 1'b0;
            qd_r[op_q].gen    <= qd_r[op_q].gen + 8'd1;
            quiet_cnt     <= '0;
            eq_cnt        <= '0;
            tot_cnt       <= '0;
            op_b          <= '0;
            state         <= C_RM_WAIT;
          end else if (req_dir) begin
            req_dir <= 1'b0;
            dr_val  <= 1'b1;
            dr_idx  <= {dir_req_q, dir_req_b[QBLK_AW-1:0]};
            state   <= C_DIR_RD;
          end else if (req_ro) begin
            req_ro      <= 1'b0;
            cmd_hold_r  <= 1'b1;      // see header: sample busy one state on
            ro_timeout  <= '0;
            state       <= C_RO_HOLD;
          end else if (req_tlm) begin
            req_tlm  <= 1'b0;
            acc_sel  <= 5'd0;
            tlm_i    <= 5'd0;
            state    <= C_TLM;
          end
        end

        // ---- install: pop blocks, write entries, publish ----------------
        C_INS_WAIT: begin
          tot_cnt <= tot_cnt + 20'd1;
          if (mig_busy || tm_cmd_val)
            quiet_cnt <= '0;
          else
            quiet_cnt <= quiet_cnt + 4'd1;
          if (quiet_cnt >= 4'd2)
            state <= C_INS_POP;       // scan cut: tier manager parked idle
          else if (tot_cnt >= 20'd100_000) begin
            st_op_err  <= 1'b1;
            blk_scan_r <= 1'b0;
            state      <= C_IDLE;
          end
        end

        C_INS_POP: begin
          pop_tier <= op_tier_l1 ? TIER_L1 : TIER_L0;
          if (pop_empty) begin        // quota exhausted mid-install
            pop_val <= 1'b0;
            if (op_b == '0) begin
              st_op_err  <= 1'b1;     // nothing popped or written
              blk_scan_r <= 1'b0;
              state      <= C_IDLE;
            end else begin
              rb_i   <= 32'(op_b) - 32'd1;
              state  <= C_INS_RB;
            end
          end else if (pop_rdy) begin
            // grant: pop_addr is combinationally valid this cycle
            pop_val <= 1'b0;
            rb_addr[op_b[3:0]] <= pop_addr;
            dw_val <= 1'b1;
            dw_idx <= {op_q, op_b};
            dw_ent <= '{default: '0,
                        valid: 1'b1,
                        tier:  op_tier_l1 ? TIER_L1 : TIER_L0,
                        owner: OWN_RESIDENT,
                        gen:   qd_r[op_q].gen,
                        mslot: 4'd0,
                        addr:  pop_addr};
            state <= C_INS_WR;
          end else begin
            pop_val <= 1'b1;          // request and wait for the grant
          end
        end

        C_INS_WR: begin
          if (dw_val && dw_rdy) begin
            dw_val <= 1'b0;
            if (32'(op_b) + 32'd1 >= 32'(op_nblk))
              state <= C_INS_PUB;
            else begin
              op_b  <= op_b + 1'b1;
              state <= C_INS_POP;
            end
          end else
            dw_val <= 1'b1;           // hold under write contention
        end

        C_INS_PUB: begin
          qd_r[op_q].enable <= 1'b1;  // atomic publish, last step
          st_install_done   <= 1'b1;
          blk_scan_r        <= 1'b0;
          state             <= C_IDLE;
        end

        C_INS_RB: begin               // invalidate the entries already written
          dw_val <= 1'b1;
          dw_idx <= {op_q, rb_i[QBLK_AW-1:0]};
          dw_ent <= '0;
          if (dw_val && dw_rdy) begin
            dw_val <= 1'b0;
            if (rb_i == 0) begin
              rb_j   <= 32'(op_b) - 32'd1;
              state  <= C_INS_RBP;
            end else
              rb_i <= rb_i - 32'd1;
          end
        end

        C_INS_RBP: begin              // push the popped blocks back (LIFO)
          if (rb_j < 0) begin
            st_op_err  <= 1'b1;
            blk_scan_r <= 1'b0;
            state      <= C_IDLE;
          end else if (!fpp_full) begin
            fpp_full <= 1'b1;
            fpp_tier <= op_tier_l1 ? TIER_L1 : TIER_L0;
            fpp_addr <= rb_addr[rb_j[3:0]];
            rb_j     <= rb_j - 32'd1;
          end
        end

        // ---- remove: quiesce, walk, return --------------------------------
        C_RM_WAIT: begin
          tot_cnt <= tot_cnt + 20'd1;
          if (mig_busy || tm_cmd_val)
            quiet_cnt <= '0;
          else
            quiet_cnt <= quiet_cnt + 4'd1;
          if (exec_idle)
            eq_cnt <= eq_cnt + 4'd1;
          else
            eq_cnt <= '0;
          if ((quiet_cnt >= 4'd2) && (eq_cnt >= 4'd8)) begin
            dr_val <= 1'b1;
            dr_idx <= {op_q, {QBLK_AW{1'b0}}};   // block 0: '0 would be a 1-bit concat operand
            state  <= C_RM_RD;
          end else if (tot_cnt >= 20'd200_000) begin
            st_op_err    <= 1'b1;     // quiesce livelock guard
            blk_scan_r   <= 1'b0;
            state        <= C_IDLE;   // blocks not returned -- see notes
          end
        end

        C_RM_RD: begin
          // clear on dv: the "hold request" else-branch must not re-issue a
          // read after the grant, or a stale-index grant during C_RM_ADV
          // could deliver the previous block's entry as this block's
          if (dr_dv) begin
            dr_val <= 1'b0;
            if (dr_ent.valid) begin
              rm_tier_r <= dr_ent.tier;
              rm_addr_r <= dr_ent.addr;
              dw_val <= 1'b1;
              dw_idx <= {op_q, op_b};
              dw_ent <= '0;           // invalidate
              state  <= C_RM_INV;
            end else
              state <= C_RM_ADV;
          end else if (dr_val && dr_rdy)
            dr_val <= 1'b0;
          else
            dr_val <= 1'b1;
        end

        C_RM_INV: begin
          if (dw_val && dw_rdy) begin
            dw_val <= 1'b0;
            state  <= C_RM_PSH;
          end else
            dw_val <= 1'b1;
        end

        C_RM_PSH: begin               // return the block to its tier pool
          if (!fpp_full) begin
            fpp_full <= 1'b1;
            fpp_tier <= rm_tier_r;
            fpp_addr <= rm_addr_r;
            state    <= C_RM_ADV;
          end
        end

        C_RM_ADV: begin
          if (op_b == BLK_LAST) begin
            st_remove_done <= 1'b1;
            blk_scan_r     <= 1'b0;
            state          <= C_IDLE;
          end else begin
            op_b  <= op_b + 1'b1;
            dr_val <= 1'b1;
            dr_idx <= {op_q, op_b + 1'b1};
            state  <= C_RM_RD;
          end
        end

        // ---- directory readback -------------------------------------------
        C_DIR_RD: begin
          if (dr_dv) begin           // clear on dv -- see C_RM_RD note
            dr_val    <= 1'b0;
            dir_ent_r <= dr_ent;
            if (dack_kind == DACK_DIR) begin
              b_v       <= 1'b1;
              dack_kind <= DACK_NONE;
            end
            state <= C_IDLE;
          end else if (dr_val && dr_rdy)
            dr_val <= 1'b0;
          else
            dr_val <= 1'b1;
        end

        // ---- sketch readout -------------------------------------------------
        C_RO_HOLD: begin
          state <= C_RO_CHK;          // cmd_hold already high; let it bite
        end

        C_RO_CHK: begin
          if (!mig_busy) begin
            // airtight: any command handshake before cmd_hold rose has
            // busy high in this cycle; later ones are gated off
            ro_hold_r <= 1'b1;
            if (!ro_tier_r) begin     // L0: one word through r1
              ro_l0_val_r  <= 1'b1;
              ro_l0_addr_r <= ro_field_r[L0_AW-1:0];
              state        <= C_RO_L0;
            end else begin            // L1: one 64-bit beat through m1
              ro_l1_arval_r <= 1'b1;
              ro_l1_addr_r  <= {ro_field_r, 3'b000};
              state         <= C_RO_L1AR;
            end
          end else begin
            ro_timeout <= ro_timeout + 20'd1;
            if (ro_timeout >= 20'd100_000) begin
              st_op_err  <= 1'b1;
              cmd_hold_r <= 1'b0;
              if (dack_kind == DACK_RO) begin
                b_v       <= 1'b1;
                dack_kind <= DACK_NONE;
              end
              state <= C_IDLE;
            end
          end
        end

        C_RO_L0: begin
          if (st_l0r_dv) begin       // clear on dv: no re-issue after grant
            ro_l0_val_r <= 1'b0;
            ro_lo_r     <= st_l0r_data;
            ro_hi_r     <= 32'h0;
            state       <= C_RO_REL;
          end else if (ro_l0_val_r && st_l0r_rdy)
            ro_l0_val_r <= 1'b0;
          else
            ro_l0_val_r <= 1'b1;
        end

        C_RO_L1AR: begin
          if (ro_l1_arval_r && st_l1_arready) begin
            ro_l1_arval_r <= 1'b0;
            ro_l1_rrdy_r  <= 1'b1;
            state         <= C_RO_L1R;
          end else
            ro_l1_arval_r <= 1'b1;
        end

        C_RO_L1R: begin
          if (st_l1_rvalid) begin
            ro_lo_r       <= st_l1_rdata[31:0];
            ro_hi_r       <= st_l1_rdata[63:32];
            ro_l1_rrdy_r  <= 1'b0;
            state         <= C_RO_REL;
          end
        end

        C_RO_REL: begin
          ro_hold_r  <= 1'b0;
          cmd_hold_r <= 1'b0;
          if (dack_kind == DACK_RO) begin
            b_v       <= 1'b1;
            dack_kind <= DACK_NONE;
          end
          state <= C_IDLE;
        end

        // ---- telemetry snapshot ----------------------------------------------
        C_TLM: begin
          if (tlm_i < 5'd24) begin
            // acc_rd is combinational on acc_sel, which smu_exec decodes as
            // {qid@4:3, ctr@2:0} = q*8+c -- NOT the register index q*6+c.
            // The value latched here was selected by the acc_sel programmed
            // one iteration ago (index 0's at the C_IDLE -> C_TLM edge), so
            // program index i+1 now.  encoding(24) truncates to 0 in 5 bits,
            // which is never latched (the next cycle parks acc_sel anyway).
            tlm_q_r[tlm_i] <= acc_rd;
            acc_sel        <= (((tlm_i + 5'd1) / 6) * 8) + ((tlm_i + 5'd1) % 6);
            tlm_i          <= tlm_i + 5'd1;
          end else begin
            tlm_g_r[0] <= mig_cnt;
            tlm_g_r[1] <= rep_ok_cnt;
            tlm_g_r[2] <= rep_drop_cnt;
            tlm_g_r[3] <= cmd_rej_cnt;
            tlm_g_r[4] <= err_cnt;
            tlm_g_r[5] <= prom_cnt;
            tlm_g_r[6] <= dem_cnt;
            tlm_g_r[7] <= scan_passes;
            tlm_g_r[8] <= l0_free_cnt;
            tlm_g_r[9] <= l1_free_cnt;
            acc_sel    <= 5'd0;            // park
            if (dack_kind == DACK_TLM) begin
              b_v       <= 1'b1;
              dack_kind <= DACK_NONE;
            end
            state <= C_IDLE;
          end
        end

        default: state <= C_IDLE;
      endcase
    end
  end

  // ------------------------------------------------------------------
  // L0 r1 bypass mux: pass the engine through unless the readout FSM
  // holds the port (engine is provably idle then -- mig_busy was sampled
  // low under cmd_hold, and only the engine drives r1).
  // ------------------------------------------------------------------
  wire l0_pt = ~ro_hold_r;
  assign st_l0r_val  = l0_pt ? eng_l0r_val  : ro_l0_val_r;
  assign st_l0r_addr = l0_pt ? eng_l0r_addr : ro_l0_addr_r;
  assign eng_l0r_rdy = l0_pt ? st_l0r_rdy   : 1'b0;
  assign eng_l0r_data = st_l0r_data;
  assign eng_l0r_dv  = l0_pt ? st_l0r_dv    : 1'b0;

  // ------------------------------------------------------------------
  // L1 m1 bypass mux: same discipline on all five channels.  The engine
  // is idle during a hold, so its aw/b traffic is quiescent; ctrl drives
  // the store-side write channels low rather than gating them back.
  // ------------------------------------------------------------------
  wire l1_pt = ~ro_hold_r;
  assign st_l1_arvalid = l1_pt ? eng_l1_arvalid : ro_l1_arval_r;
  assign st_l1_araddr  = l1_pt ? eng_l1_araddr  : ro_l1_addr_r;
  assign eng_l1_arready = l1_pt ? st_l1_arready : 1'b0;
  assign eng_l1_rvalid = l1_pt ? st_l1_rvalid   : 1'b0;
  assign eng_l1_rdata  = st_l1_rdata;
  assign st_l1_rready  = l1_pt ? eng_l1_rready  : ro_l1_rrdy_r;

  assign st_l1_awvalid = l1_pt ? eng_l1_awvalid : 1'b0;
  assign st_l1_awaddr  = l1_pt ? eng_l1_awaddr  : 32'h0;
  assign st_l1_awdata  = l1_pt ? eng_l1_awdata  : 64'h0;
  assign st_l1_awstrb  = l1_pt ? eng_l1_awstrb  : 8'h0;
  assign eng_l1_awready = l1_pt ? st_l1_awready : 1'b0;
  assign eng_l1_bvalid = l1_pt ? st_l1_bvalid   : 1'b0;
  assign st_l1_bready  = l1_pt ? eng_l1_bready  : 1'b0;

endmodule
