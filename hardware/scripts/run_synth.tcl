# SketchSoC out-of-context synthesis (Alveo V80, xcv80-lsva4737-2MHP-e-S).
#
# Usage:  vivado -mode batch -source run_synth.tcl
#         (from scripts/, or pass -tclargs <rtl_dir> <build_dir>)
#
# Synthesizes sketchsoc_top out-of-context with a 220 MHz clock constraint
# (4.545 ns) -- the paper's FPGA configuration frequency (Sec. 4.1).
# Prints utilization + timing summary; writes reports and a checkpoint
# under build/synth.
#
# NOTE: the XCV80 (Versal HBM) part requires Vivado 2023.2 or newer; the
# paper's prototype was built with Vivado 2023.2.

set rtl_dir   [lindex $argv 0]
set build_dir [lindex $argv 1]
if {$rtl_dir eq ""}   { set rtl_dir   "[file dirname [info script]]/../rtl" }
if {$build_dir eq ""} { set build_dir "[file dirname [info script]]/../build/synth" }
file mkdir $build_dir

set part xcv80-lsva4737-2MHP-e-S

# package first (everything imports it), then the remaining RTL
read_verilog -sv $rtl_dir/sketchsoc_pkg.sv
foreach f [lsort [glob -nocomplain $rtl_dir/*.sv]] {
  if {[file tail $f] ne "sketchsoc_pkg.sv"} {
    read_verilog -sv $f
  }
}

synth_design -top sketchsoc_top -part $part -mode out_of_context

# 220 MHz system clock
create_clock -period 4.545 -name sys_clk [get_ports clk]

# multicycle hash cone: smu_exec's ST_HASH holds w_row/w_key for HASH_WAIT
# cycles while the combinational row_hash (CRC32 XOR network / serial fmix
# multiplies) settles, then captures it into hash_reg/rank_reg/fp_reg/
# bit_reg.  MUST match HASH_WAIT in rtl/smu_exec.sv.
set HASH_WAIT 8
set mcp_cells [get_cells -quiet -hier -regexp {.*(hash_reg|rank_reg|fp_reg|bit_reg)_reg\[\d+\]}]
puts "== MCP cells matched: [llength $mcp_cells] (expect 61) =="
if {[llength $mcp_cells] == 0} {
  puts "ERROR: multicycle pattern matched no cells -- register names changed?"
  exit 1
}
set_multicycle_path $HASH_WAIT -setup -to $mcp_cells
set_multicycle_path [expr {$HASH_WAIT - 1}] -hold -to $mcp_cells

opt_design
place_design
route_design

report_utilization    -file $build_dir/utilization.rpt
report_timing_summary -file $build_dir/timing_summary.rpt
report_clocks         -file $build_dir/clocks.rpt
write_checkpoint -force $build_dir/sketchsoc_top_routed.dcp

# console summary
puts "== utilization (top) =="
report_utilization -return_string
puts "== timing summary =="
report_timing_summary -return_string

set wns [get_property SLACK [get_timing_paths -max_paths 1 -nworst 1 -setup]]
puts "== WNS: $wns ns =="
if {$wns < 0} {
  puts "== TIMING FAILED @ 220 MHz =="
} else {
  puts "== TIMING CLOSED @ 220 MHz =="
}
