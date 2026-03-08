#!/bin/bash
# Admittance Controller Comparison Demo
#
# This script runs the full comparison demo and opens the result plot.
#
# Usage:
#   ./run_comparison.sh              # default: 20s per phase, with RViz
#   ./run_comparison.sh 15           # custom phase duration (seconds)
#   ./run_comparison.sh 20 false     # no RViz (headless)

set -e

PHASE_DURATION="${1:-20}"
GUI="${2:-true}"
OUTPUT_DIR="/tmp/admittance_comparison"

echo "=============================================="
echo " Admittance Controller Comparison Demo"
echo "=============================================="
echo " Phase duration:  ${PHASE_DURATION}s per phase"
echo " Total duration:  ~$((PHASE_DURATION * 2 + 15))s"
echo " Output dir:      ${OUTPUT_DIR}"
echo " RViz:            ${GUI}"
echo "=============================================="
echo ""
echo "The demo will (NO trajectory commands, force-only):"
echo "  1. Start robot + admittance controller (robot at home position)"
echo "  2. Simulated 50N sinusoidal force on z-axis as disturbance"
echo "  3. Phase 1: HIGH stiffness (K=10000) -> robot barely moves"
echo "  4. Phase 2: NORMAL stiffness (K=200)  -> robot oscillates visibly"
echo "  5. Generate comparison plots"
echo ""
echo "Press Ctrl+C to abort. Starting in 3s..."
sleep 3

ros2 launch ros2_control_demo_example_16 r6bot_comparison_demo.launch.py \
    phase_duration:="${PHASE_DURATION}" \
    gui:="${GUI}" \
    output_dir:="${OUTPUT_DIR}"

echo ""
echo "=============================================="
echo " Demo complete!"
echo " Results saved to: ${OUTPUT_DIR}/"
echo "   - admittance_comparison.png  (comparison plot)"
echo "   - phase1_rigid.csv           (high stiffness data)"
echo "   - phase2_compliant.csv       (low stiffness data)"
echo "=============================================="

# Try to open the plot
if command -v xdg-open &> /dev/null; then
    xdg-open "${OUTPUT_DIR}/admittance_comparison.png" 2>/dev/null || true
elif command -v eog &> /dev/null; then
    eog "${OUTPUT_DIR}/admittance_comparison.png" 2>/dev/null || true
fi
