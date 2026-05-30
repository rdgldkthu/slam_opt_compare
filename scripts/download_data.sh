#!/usr/bin/env bash
# Download the Manhattan 3500-node pose graph dataset
# Source: https://lucacarlone.mit.edu/datasets/
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
DATA_DIR="$SCRIPT_DIR/../data"
mkdir -p "$DATA_DIR"
DATA_DIR="$(cd "$DATA_DIR" && pwd)"

URL="https://www.dropbox.com/s/gmdzo74b3tzvbrw/input_M3500_g2o.g2o?dl=1"
OUT="$DATA_DIR/manhattan3500.g2o"

echo "[download] Fetching manhattan3500 dataset (M3500 g2o format) ..."
curl -L "$URL" -o "$OUT"

echo "[download] Done. Dataset at: $DATA_DIR/manhattan3500.g2o"
