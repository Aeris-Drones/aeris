#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"

bridge_pid=""
publisher_pid=""

cleanup() {
  if [[ -n "${publisher_pid}" ]] && kill -0 "${publisher_pid}" 2>/dev/null; then
    kill "${publisher_pid}" 2>/dev/null || true
  fi
  if [[ -n "${bridge_pid}" ]] && kill -0 "${bridge_pid}" 2>/dev/null; then
    kill "${bridge_pid}" 2>/dev/null || true
  fi
}

trap cleanup EXIT INT TERM

echo "[smoke] Starting mock rosbridge on ws://localhost:9090"
python3 "${ROOT_DIR}/tools/sim/mock_rosbridge.py" &
bridge_pid=$!

sleep 1

echo "[smoke] Publishing fused detections to /detections/fused"
python3 "${ROOT_DIR}/tools/sim/publish_perception.py" --fused &
publisher_pid=$!

cat <<'EOF'
[smoke] Viewer fused-detection smoke path is now live.
1. In another terminal: cd software/viewer && npm run dev
2. Open the viewer and confirm:
   - Detection panel populates with live fused detections
   - Thermal/acoustic/gas counters update in real time
   - Confirm/dismiss actions persist by candidate ID
   - Map markers and locate interactions stay synchronized
Press Ctrl+C to stop the smoke publishers.
EOF

wait
