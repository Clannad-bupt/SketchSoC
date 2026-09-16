#!/bin/bash
# SketchSoC synthesis runner (BUPT server, Vivado 2021.2).
# Out-of-context synthesis of sketchsoc_top for xcu280 @ 220 MHz.
set -e
source /tools/Xilinx/Vivado/2021.2/settings64.sh
TOP="$(cd "$(dirname "$0")/.." && pwd)"
mkdir -p "$TOP/build/synth"
cd "$TOP/build/synth"
vivado -mode batch -source "$TOP/scripts/run_synth.tcl" \
  -tclargs "$TOP/rtl" "$TOP/build/synth"
