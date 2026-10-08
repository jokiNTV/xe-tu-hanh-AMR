#!/usr/bin/env bash
set -euo pipefail
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WEB_PID=/tmp/ht_v1_amr_b2_gui.pid
BRIDGE_PID=/tmp/ht_v1_amr_b2_bridge.pid
CORE_PID=/tmp/ht_v1_amr_b2/core.pid

say(){ printf '[%s] %s\n' "$(date +%H:%M:%S)" "$*"; }

if [[ -f "$CORE_PID" ]]; then
  say "Yêu cầu runtime dừng có kiểm soát..."
  /usr/bin/python3 "$BASE/HT_V1_CORE_AMR.py" stop || true
  END=$((SECONDS+45))
  while [[ -f "$CORE_PID" ]] && (( SECONDS < END )); do sleep .5; done
  if [[ -f "$CORE_PID" ]]; then
    say "Runtime chưa xác nhận dừng. Không force-kill tầng robot."
    exit 4
  fi
fi

for pair in "$BRIDGE_PID:Bridge giám sát" "$WEB_PID:Giao diện"; do
  f="${pair%%:*}"; name="${pair#*:}"
  [[ -f "$f" ]] || continue
  p="$(cat "$f" 2>/dev/null || true)"
  if [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null; then
    say "Dừng $name PID=$p"
    kill -INT "$p" 2>/dev/null || true
    sleep .8
    kill -TERM "$p" 2>/dev/null || true
  fi
  rm -f "$f"
done
say "HT_V1.1 đã dừng."
