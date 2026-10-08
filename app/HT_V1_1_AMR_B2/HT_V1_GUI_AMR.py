#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
import os
import signal
import socket
import subprocess
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

from fastapi import Body, FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, JSONResponse, Response
from fastapi.staticfiles import StaticFiles

BASE = Path(__file__).resolve().parent
STATIC = BASE / "static"
RUNTIME = Path("/tmp/ht_v1_amr_b2")
STATE_FILE = RUNTIME / "state.json"
CORE_PID_FILE = RUNTIME / "core.pid"
CAM_FRAME = Path("/tmp/amr_b2_camera_broker/latest.jpg")
CAM_STATUS = Path("/tmp/amr_b2_camera_broker/status.json")
UDP_HOST, UDP_PORT = "127.0.0.1", 8767
VERSION = "HT_V1_1_AMR_B2"

app = FastAPI(title=VERSION, docs_url=None, redoc_url=None)
app.mount("/static", StaticFiles(directory=STATIC), name="static")
_lock = threading.Lock()
_latest: dict[str, Any] = {}
_latest_map: dict[str, Any] | None = None
_last_bridge = 0.0
_thread_started = False


def local_ips() -> set[str]:
    ips = {"127.0.0.1", "::1"}
    try:
        out = subprocess.check_output(["hostname", "-I"], text=True, timeout=1)
        ips.update(x for x in out.split() if x)
    except Exception:
        pass
    return ips

LOCAL_IPS = local_ips()


def is_local(host: str | None) -> bool:
    return bool(host and host in LOCAL_IPS)


def core_state() -> dict[str, Any]:
    try:
        d = json.loads(STATE_FILE.read_text(encoding="utf-8"))
        if isinstance(d, dict):
            age = max(0.0, time.time() - float(d.get("timestamp") or 0))
            d["age_s"] = round(age, 2)
            return d
    except Exception:
        pass
    return {"status": "DA_TAT", "stage": "DA_TAT", "message": "Hệ thống chưa khởi động", "pid": None, "age_s": None}


def core_pid() -> int | None:
    try:
        pid = int(CORE_PID_FILE.read_text().strip())
        os.kill(pid, 0)
        return pid
    except Exception:
        return None


def udp_receiver() -> None:
    global _latest, _latest_map, _last_bridge
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    s.bind((UDP_HOST, UDP_PORT))
    while True:
        try:
            raw, _ = s.recvfrom(262144)
            d = json.loads(raw.decode("utf-8", errors="replace"))
            if not isinstance(d, dict):
                continue
            m = d.pop("map", None)
            with _lock:
                if isinstance(m, dict):
                    _latest_map = m
                _latest = d
                _last_bridge = time.monotonic()
        except Exception:
            time.sleep(0.05)


def ensure_thread() -> None:
    global _thread_started
    if _thread_started:
        return
    threading.Thread(target=udp_receiver, daemon=True, name="ht_v1_gui_udp").start()
    _thread_started = True


@app.on_event("startup")
def on_startup() -> None:
    ensure_thread()


def snapshot(include_map: bool = False) -> dict[str, Any]:
    with _lock:
        d = dict(_latest)
        m = _latest_map
        age = None if _last_bridge <= 0 else time.monotonic() - _last_bridge
    if not d:
        d = {
            "seq": 0, "ros_live": False, "system": "WAITING ROS", "robot_state": "OFFLINE",
            "mission": "IDLE", "esp32": "OFFLINE", "sensor1": "UNKNOWN", "sensor2": "UNKNOWN",
            "pose": None, "velocity": {"linear": 0.0, "angular": 0.0}, "global_path": [], "local_path": [],
            "scan_points": [], "costmap_obstacles": [], "global_costmap_obstacles": [], "hardware": {},
            "supervisor": {}, "mission_monitor": {}, "event_log": [], "topics": {}, "route_markers": [],
        }
    d["bridge_live"] = bool(age is not None and age < 1.5)
    d["bridge_age_s"] = None if age is None else round(age, 2)
    d["core"] = core_state()
    if include_map and m is not None:
        d["map"] = m
    return d


def supervisor_command(cmd: str, timeout: float = 1.0) -> tuple[bool, str]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(cmd.encode(), ("127.0.0.1", 5005))
        data, _ = s.recvfrom(4096)
        return True, data.decode(errors="replace").strip()
    except Exception as exc:
        return False, str(exc)
    finally:
        s.close()


def spawn_core() -> tuple[bool, str]:
    if core_pid():
        return False, "Hệ thống đang khởi động hoặc đã hoạt động."
    log_dir = Path.home() / "AMR_LOGS"
    log_dir.mkdir(parents=True, exist_ok=True)
    launch_log = log_dir / "HT_V1_CORE_LAUNCH.log"
    f = launch_log.open("ab", buffering=0)
    try:
        p = subprocess.Popen(["/usr/bin/python3", "-u", str(BASE / "HT_V1_CORE_AMR.py"), "start"], stdout=f, stderr=subprocess.STDOUT, start_new_session=True)
        time.sleep(0.25)
        if p.poll() is not None:
            return False, "Bộ điều phối dừng ngay khi khởi động. Xem nhật ký kỹ thuật."
        return True, f"Đã bắt đầu khởi động hệ thống. PID={p.pid}"
    except Exception as exc:
        return False, f"Không khởi động được hệ thống: {exc}"


def signal_core(emergency: bool = False) -> tuple[bool, str]:
    pid = core_pid()
    if not pid:
        return False, "Hệ thống chưa chạy."
    try:
        os.kill(pid, signal.SIGUSR1 if emergency else signal.SIGTERM)
        return True, "Đã gửi lệnh DỪNG KHẨN." if emergency else "Đã gửi yêu cầu dừng hệ thống."
    except Exception as exc:
        return False, f"Không gửi được lệnh dừng: {exc}"


def run_actuator(action: str) -> dict[str, Any]:
    try:
        cp = subprocess.run(["/usr/bin/python3", str(BASE / "HT_V1_ACT_AMR.py"), action], stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, timeout=6)
        lines = [x for x in (cp.stdout or "").splitlines() if x.strip()]
        data = json.loads(lines[-1]) if lines else {"ok": False, "message": "Cơ cấu không trả lời"}
        return data
    except subprocess.TimeoutExpired:
        return {"ok": False, "message": "Cơ cấu phản hồi quá chậm"}
    except Exception as exc:
        return {"ok": False, "message": f"Lỗi điều khiển cơ cấu: {exc}"}


def camera_status() -> dict[str, Any]:
    try:
        d = json.loads(CAM_STATUS.read_text(encoding="utf-8"))
        age = max(0.0, time.time() - float(d.get("timestamp") or 0))
        return {**d, "age_s": round(age, 2), "live": d.get("state") == "LIVE" and age < 1.5 and CAM_FRAME.exists()}
    except Exception:
        return {"state": "OFFLINE", "age_s": None, "live": False}


def reset_localization_a_h() -> tuple[bool, str]:
    """Call the already-running Home Localizer service. This never commands robot motion."""
    try:
        cp = subprocess.run(
            [
                "ros2", "service", "call",
                "/amr/reset_to_A_H",
                "std_srvs/srv/Trigger",
                "{}",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=8,
        )
        out = (cp.stdout or "").strip()
        low = out.lower().replace(" ", "")
        explicit_ok = ("success=true" in low) or ("success:true" in low)
        if cp.returncode != 0:
            return False, f"Không gọi được Home Localizer: {out[-400:] or 'ros2 service call thất bại'}"
        if not explicit_ok:
            return False, f"Home Localizer chưa xác nhận thành công: {out[-400:] or 'không có phản hồi'}"
        return True, "Đã đặt lại định vị trên bản đồ về A_H."
    except subprocess.TimeoutExpired:
        return False, "Home Localizer phản hồi quá chậm."
    except Exception as exc:
        return False, f"Lỗi khi đặt lại định vị A_H: {exc}"


@app.get("/")
def index():
    return FileResponse(STATIC / "index.html")


@app.get("/api/status")
def api_status(request: Request):
    return {
        "ok": True,
        "local_operator": is_local(request.client.host if request.client else None),
        "core": core_state(),
        "core_pid": core_pid(),
        "bridge_live": snapshot(False).get("bridge_live", False),
        "version": VERSION,
        "time": datetime.now().isoformat(timespec="seconds"),
    }


@app.post("/api/bat-dau")
def api_start(request: Request):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})
    ok, msg = spawn_core()
    return {"ok": ok, "message": msg}


@app.post("/api/dung-nhan")
def api_stop_jobs(request: Request):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})
    ok, msg = supervisor_command("WORK_OFF", 0.8)
    return {"ok": ok, "message": "Đã tạm dừng nhận nhiệm vụ mới." if ok else f"Bộ giám sát chưa phản hồi: {msg}"}


@app.post("/api/tiep-tuc")
def api_resume(request: Request):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})
    if core_state().get("status") not in ("SAN_SANG", "CAN_KHOI_PHUC"):
        return {"ok": False, "message": "Hệ thống chưa ở trạng thái cho phép tiếp tục."}
    ok_st, st_text = supervisor_command("STATUS", 0.8)
    if not ok_st:
        return {"ok": False, "message": "Bộ giám sát chưa phản hồi."}
    fields = {}
    for tok in st_text.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1); fields[k] = v
    if fields.get("state") != "IDLE":
        return {"ok": False, "message": "Nhiệm vụ chưa về trạng thái chờ."}
    if fields.get("sensor1") != "CLEAR":
        return {"ok": False, "message": "Cảm biến 1 phải KHÔNG CÓ VẬT trước khi tiếp tục nhận nhiệm vụ."}
    ok, msg = supervisor_command("WORK_ON", 0.8)
    return {"ok": ok, "message": "Đã tiếp tục nhận nhiệm vụ B2." if ok else f"Bộ giám sát chưa phản hồi: {msg}"}


@app.post("/api/dung-khan")
def api_emergency(request: Request):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})
    supervisor_command("EMERGENCY_STOP", 0.25)
    ok, msg = signal_core(True)
    return {"ok": ok, "message": msg + " E-stop phần cứng vẫn là ưu tiên cao nhất."}


@app.post("/api/tat-he-thong")
def api_stop_system(request: Request):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})
    ok, msg = signal_core(False)
    return {"ok": ok, "message": msg}


@app.post("/api/dat-lai-a-h")
def api_reset_a_h(request: Request, payload: dict[str, Any] = Body(default={})):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})

    if not bool(payload.get("confirm_physical_ah")):
        return {"ok": False, "message": "Chưa xác nhận robot đang ở đúng mốc vật lý A_H."}

    d = snapshot(False)
    core = d.get("core") or {}
    sup = d.get("supervisor") or {}
    vel = d.get("velocity") or {}

    if not d.get("bridge_live"):
        return {"ok": False, "message": "Chưa có telemetry ổn định; không đặt lại định vị."}

    if core.get("status") != "SAN_SANG":
        return {"ok": False, "message": "Chỉ được đặt lại A_H khi hệ thống đang SẴN SÀNG và không chạy nhiệm vụ."}

    if sup.get("state") != "IDLE":
        return {"ok": False, "message": "Nhiệm vụ chưa ở trạng thái chờ; không đặt lại A_H."}

    try:
        v = abs(float(vel.get("linear") or 0.0))
        w = abs(float(vel.get("angular") or 0.0))
    except Exception:
        return {"ok": False, "message": "Không xác minh được vận tốc robot."}

    if v > 0.01 or w > 0.02:
        return {
            "ok": False,
            "message": f"Robot chưa đứng yên (v={v:.3f} m/s, w={w:.3f} rad/s).",
        }

    ok, msg = reset_localization_a_h()
    return {"ok": ok, "message": msg}


@app.post("/api/co-cau/{action}")
def api_actuator(action: str, request: Request):
    if not is_local(request.client.host if request.client else None):
        return JSONResponse(status_code=403, content={"ok": False, "message": "Máy từ xa chỉ được giám sát."})
    mapping = {
        "trang-thai": "status", "dat-lai": "reset_fault", "dung": "stop",
        "mo-kep": "grip_open", "dong-kep": "grip_close", "nang": "lift_up", "ha": "lift_down",
    }
    if action not in mapping:
        return {"ok": False, "message": "Lệnh cơ cấu không hợp lệ."}
    return run_actuator(mapping[action])


@app.get("/api/camera/status")
def api_camera_status():
    return camera_status()


@app.get("/api/camera/frame")
def api_camera_frame():
    st = camera_status()
    if not st.get("live"):
        return JSONResponse(status_code=503, content={"ok": False, "message": "Camera chưa sẵn sàng"})
    try:
        return Response(content=CAM_FRAME.read_bytes(), media_type="image/jpeg", headers={"Cache-Control": "no-store"})
    except Exception:
        return JSONResponse(status_code=503, content={"ok": False, "message": "Không đọc được ảnh camera"})


@app.websocket("/ws/telemetry")
async def ws_telemetry(ws: WebSocket):
    await ws.accept()
    sent_map_revision = None
    try:
        while True:
            with _lock:
                rev = None if _latest_map is None else _latest_map.get("revision")
            include_map = rev is not None and rev != sent_map_revision
            d = snapshot(include_map)
            if include_map:
                sent_map_revision = rev
            await ws.send_text(json.dumps(d, ensure_ascii=False, separators=(",", ":")))
            await asyncio.sleep(0.2)
    except (WebSocketDisconnect, asyncio.CancelledError):
        return
    except Exception:
        try: await ws.close(code=1011)
        except Exception: pass


if __name__ == "__main__":
    import uvicorn
    uvicorn.run("HT_V1_GUI_AMR:app", host="0.0.0.0", port=8080, reload=False, ws="websockets")
