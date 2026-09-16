#!/bin/bash
cd ~/sketchsoc_fpga/sketchsoc_rtl
for t in tb_foundations tb_smu_exec tb_mig_engine tb_tier_mgr tb_ctrl_plane tb_integration; do
  echo "===== $t ====="
  ./scripts/run_sim.sh $t 2>&1 | grep -E 'FAIL|ERRORS|PASSED'
done
