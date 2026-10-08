#!/usr/bin/env bash
set -euo pipefail
BASE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PORT="${AMR_GUI_PORT:-8080}"
WEB_PID=/tmp/ht_v1_amr_b2_gui.pid
BRIDGE_PID=/tmp/ht_v1_amr_b2_bridge.pid
WEB_LOG="$HOME/AMR_LOGS/HT_V1_GUI.log"
BRIDGE_LOG="$HOME/AMR_LOGS/HT_V1_BRIDGE.log"
mkdir -p "$HOME/AMR_LOGS"
cd "$BASE"

# Nạp ROS một lần để GUI sinh tiến trình CORE luôn có đúng môi trường ROS 2.
set +u
source /opt/ros/humble/setup.bash
[[ -f "$HOME/amr_ws/install/setup.bash" ]] && source "$HOME/amr_ws/install/setup.bash"
set -u
export ROS_DOMAIN_ID=42
export FASTDDS_BUILTIN_TRANSPORTS=UDPv4

alive(){ local f="$1"; [[ -f "$f" ]] || return 1; local p; p="$(cat "$f" 2>/dev/null || true)"; [[ -n "$p" ]] && kill -0 "$p" 2>/dev/null; }

# Web Python: ưu tiên venv của HT_V1, sau đó tận dụng venv GUI cũ đã có trên MiniPC.
if [[ -x "$BASE/.venv/bin/python" ]]; then
  PY="$BASE/.venv/bin/python"
elif [[ -x "$HOME/Downloads/AMR_B2_FINAL_RC1_2_V1_6_3_STATIC_ASSET_FIX/.venv/bin/python" ]]; then
  PY="$HOME/Downloads/AMR_B2_FINAL_RC1_2_V1_6_3_STATIC_ASSET_FIX/.venv/bin/python"
elif [[ -x "$HOME/Downloads/AMR_B2_FINAL_RC1_1_V1_6_2_START_FIX/.venv/bin/python" ]]; then
  PY="$HOME/Downloads/AMR_B2_FINAL_RC1_1_V1_6_2_START_FIX/.venv/bin/python"
else
  echo "[HT_V1] Chưa có môi trường web. Đang tạo tự động..."
  python3 -m venv "$BASE/.venv"
  "$BASE/.venv/bin/pip" install -r "$BASE/requirements.txt"
  PY="$BASE/.venv/bin/python"
fi

"$PY" - <<'PY'
for m in ('fastapi','uvicorn','websockets'): __import__(m)
print('[HT_V1] Thư viện giao diện: ĐẠT')
PY

# GUI chỉ có một tiến trình.
if ! alive "$WEB_PID"; then
  rm -f "$WEB_PID"
  nohup "$PY" -m uvicorn HT_V1_GUI_AMR:app --host 0.0.0.0 --port "$PORT" --ws websockets >"$WEB_LOG" 2>&1 &
  echo $! > "$WEB_PID"
fi
WPID="$(cat "$WEB_PID")"
for _ in $(seq 1 60); do
  if curl -fsS "http://127.0.0.1:$PORT/api/status" >/dev/null 2>&1; then break; fi
  kill -0 "$WPID" 2>/dev/null || { echo "[HT_V1] Giao diện dừng bất thường"; tail -80 "$WEB_LOG"; exit 11; }
  sleep .2
done
curl -fsS "http://127.0.0.1:$PORT/api/status" >/dev/null || { echo "[HT_V1] Giao diện chưa sẵn sàng"; exit 12; }

# Bridge giám sát chỉ đọc ROS; có thể chạy trước runtime robot.
if ! alive "$BRIDGE_PID"; then
  rm -f "$BRIDGE_PID"
  set +u
  source /opt/ros/humble/setup.bash
  [[ -f "$HOME/amr_ws/install/setup.bash" ]] && source "$HOME/amr_ws/install/setup.bash"
  set -u
  export ROS_DOMAIN_ID=42
  export FASTDDS_BUILTIN_TRANSPORTS=UDPv4
  nohup /usr/bin/python3 -u "$BASE/HT_V1_BRIDGE_AMR.py" >"$BRIDGE_LOG" 2>&1 &
  echo $! > "$BRIDGE_PID"
fi

URL="http://127.0.0.1:$PORT/"
IP="$(hostname -I 2>/dev/null | awk '{print $1}')"
echo "============================================================"
echo " HT_V1.1 AMR B2 - GIAO DIỆN ĐÃ SẴN SÀNG"
echo " Nguyễn Thái Vinh"
echo " 1 LỆNH + 1 NÚT"
echo " Điều khiển tại máy: $URL"
[[ -n "${IP:-}" ]] && echo " Giám sát từ mạng: http://$IP:$PORT/"
echo " Bước tiếp theo: nhấn BẮT ĐẦU LÀM VIỆC B2 trên GUI"
echo "============================================================"

if command -v google-chrome >/dev/null 2>&1; then
  nohup google-chrome --new-window "$URL" >/dev/null 2>&1 &
elif command -v google-chrome-stable >/dev/null 2>&1; then
  nohup google-chrome-stable --new-window "$URL" >/dev/null 2>&1 &
elif command -v xdg-open >/dev/null 2>&1; then
  nohup xdg-open "$URL" >/dev/null 2>&1 &
fi
