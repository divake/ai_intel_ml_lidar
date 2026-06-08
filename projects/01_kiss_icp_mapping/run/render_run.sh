#!/bin/bash
# Re-render an exported dataset into a dense, intensity-colored map:
#   <name>_render/  ->  map_dense.ply, map_topdown_intensity.png,
#                       map_side_intensity.png, map_interactive.html
# Does NOT touch the dataset. Re-run with any voxel as often as you like.
# Usage: bash render_run.sh <run_name> [voxel_m]   (default voxel 0.02)
set -e
NAME=${1:-lab_room1}
VOXEL=${2:-0.02}
PROJ=/home/nus-ai/divek_nus/ml_lidar/projects/01_kiss_icp_mapping
# conda python: needs numpy + matplotlib + plotly (NOT rclpy), so NOT /usr/bin/python3
PY=/home/nus-ai/miniconda3/envs/intel_ai/bin/python
[ -x "$PY" ] || PY=python3
"$PY" "$PROJ/run/render_map.py" \
  "$PROJ/results/${NAME}_dataset" "$PROJ/results/${NAME}_render" --voxel "$VOXEL"
echo "Render -> $PROJ/results/${NAME}_render"
