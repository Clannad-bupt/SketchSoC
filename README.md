# SketchSoC — Hardware Artifact

Synthesizable SystemVerilog implementation of the core mechanisms of the
SketchSoC paper: *Query-Driven In-Switch Telemetry with Live
Reconfiguration and Tiered State*.

Target: **AMD Alveo V80** (`xcv80-lsva4737-2MHP-e-S`, Versal HBM — the
paper's §4.1 prototype card), 220 MHz, Vivado 2023.2+. Verified with the
Vivado simulator (`xsim`). Timing closure was proxy-validated out-of-context
on `xcu280-fsvh2892-2L-e` @ 220 MHz (WNS = +0.189 ns; the RTL is
device-independent inference-only RTL), because the available build machine
runs Vivado 2021.2, which lacks XCV80 support — re-run
`hardware/scripts/run_synth.sh` with Vivado 2023.2+ for the on-target
numbers.

## What is implemented

| Mechanism (paper) | Module(s) |
|---|---|
| Multi-query sketch execution (§3.4): CMS / Heavy-Hitter / Bloom / HLL, 4 query slots | `hardware/rtl/smu_exec.sv` |
| Tiered state (§3.4.3): L0 on-chip SRAM, L1 backing store over AXI, block directory | `l0_store.sv`, `l1_arb2.sv`, `state_directory.sv` |
| Heat-driven migration (§3.6): EWMA heat, promote/demote, freeze–copy–commit with exactly-once replay | `heat_table.sv`, `tier_mgr.sv`, `mig_engine.sv` |
| Live reconfiguration (§3.5): install / remove / readout over AXI4-Lite, no resynthesis | `ctrl_plane.sv` |
| Work arbitration & quiesce backpressure (§3.6.3) | `work_mux.sv` |
| Integration top | `sketchsoc_top.sv` |

## Repository layout

```
hardware/
  rtl/        synthesizable SystemVerilog (11 modules + package)
  tb/         6 xsim testbenches (unit → integration)
  scripts/    run_sim.sh / regress_all.sh / run_synth.sh(.tcl)
  docs/       board_notes.md — integration & register-map guide (中文)
p4-baseline/
  pure_cms_rdma.p4 + table scripts   Tofino count-min baseline
  logs/                            original build/run logs
```

## Quick start

```bash
# simulation (Vivado 2021.2 xsim)
hardware/scripts/run_sim.sh tb_ctrl_plane      # one testbench
hardware/scripts/regress_all.sh                # full 6-TB regression

# out-of-context synthesis + place & route, xcu280 @ 220 MHz
hardware/scripts/run_synth.sh                  # reports in build/synth/
```

Expected result: all six testbenches print `ALL TESTS PASSED`
(`tb_foundations`, `tb_smu_exec`, `tb_mig_engine`, `tb_tier_mgr`,
`tb_ctrl_plane`, `tb_integration`).

## Host interface

`sketchsoc_top` exposes an AXI4-Lite control port (query install /
remove, pool seeding, sketch readout, telemetry), a 256-bit packet
ingress stream with quiesce backpressure, and an AXI master for the L1
backing store (HBM on U280 — see `hardware/docs/board_notes.md` §4.4
for the clock/width conversion notes). The full register map and the
minimal host driver sequence are documented in
`hardware/docs/board_notes.md` §6–§7.

## Notes

* `p4-baseline/` is the original repository's Tofino count-min/RDMA
  baseline, kept for reference; it is not part of the FPGA build.
* Known limitations of this v1 (install guard, heat inheritance on
  reinstall, HH candidate readout, AXI RESP handling) are listed in
  `hardware/docs/board_notes.md` §8.
