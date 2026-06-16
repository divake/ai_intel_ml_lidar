#!/usr/bin/env bash
# Pull the rendered demo videos FROM this server TO your laptop.
# RUN THIS ON YOUR LAPTOP (not the server). Fill in the two blanks first.
set -u

SERVER="divake@<SERVER_IP>"          # <-- the box these renders live on
DEST="$HOME/ml_lidar_demo"           # <-- where to drop them on your laptop
SRC="/ssd_4TB/divake/ml_lidar/projects/04_demo_viz/out"

mkdir -p "$DEST"
# -z compress (link is the bottleneck), only the light artifacts (mp4/png), skip the big caches
rsync -a -z --info=progress2 \
  --include='*.mp4' --include='*.png' --exclude='*' \
  -e "ssh" "$SERVER:$SRC/" "$DEST/"
echo "Done -> $DEST"
ls -lh "$DEST"
