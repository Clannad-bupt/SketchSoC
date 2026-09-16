#!/bin/bash
# SketchSoC xsim regression (BUPT server, Vivado 2021.2).
#
# Usage:  ./run_sim.sh [tb_module]      (default: tb_foundations)
#
# Compiles the package first (everything imports it), then the remaining
# RTL, then the testbench; elaborates and runs.  Exits non-zero on
# compile/elab failure; the TB itself prints PASS/FAIL and $finish.

set -e

source /tools/Xilinx/Vivado/2021.2/settings64.sh

TOP="$(cd "$(dirname "$0")/.." && pwd)"
TB=${1:-tb_foundations}

# package first, then the rest of the RTL, then the testbench
PKG="$TOP/rtl/sketchsoc_pkg.sv"
REST=$(ls "$TOP"/rtl/*.sv | grep -v sketchsoc_pkg.sv | sort)

mkdir -p "$TOP/build/xsim"
cd "$TOP/build/xsim"

echo "== xvlog =="
xvlog -sv "$PKG" $REST "$TOP/tb/$TB.sv"

echo "== xelab =="
xelab -debug off "$TB" -s "${TB}_sim"

echo "== xsim =="
xsim "${TB}_sim" -R
