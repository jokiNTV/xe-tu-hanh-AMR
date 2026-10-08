#!/usr/bin/env python3
from __future__ import annotations

import fcntl
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

BASE = Path(__file__).resolve().parent
CFG = yaml.safe_load((BASE / "HT_V1_CFG_AMR.yaml").read_text(encoding="utf-8"))
HOME = Path.home()
RUNTIME_DIR = Path("/tmp/ht_v1_amr_b2")
RUNTIME_DIR.mkdir(parents=True, exist_ok=True)
STATE_FILE = RUNTIME_DIR / "state.json"
PID_FILE = RUNTIME_DIR / "core.pid"
LOCK_FILE = RUNTIME_DIR / "core.lock"
STOP_REQUESTED = threading.Event()
EMERGENCY_REQUESTED = threading.Event()

def _build_ros_env() -> dict[str, str]:
    env = os.environ.copy()
    try:
        cp = subprocess.run([
            "bash", "-lc",
            "source /opt/ros/humble/setup.bash; "
            "source $HOME/amr_ws/install/setup.bash; env -0"
        ], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL, timeout=6)
        if cp.returncode == 0 and cp.stdout:
            for item in cp.stdout.decode(errors="replace").split("\0"):
                if "=" in item:
                    k, v = item.split("=", 1); env[k] = v
    except Exception:
        pass
    env["ROS_DOMAIN_ID"] = str(CFG["he_thong"]["ros_domain_id"])
    env["FASTDDS_BUILTIN_TRANSPORTS"] = "UDPv4"
    return env

ROS_ENV = _build_ros_env()

STABLE = CFG["tep_on_dinh"]
HW = CFG["phan_cung"]
TH = CFG["nguong"]
HASHES = CFG.get("hash_on_dinh", {})

PROCS: list[dict[str, Any]] = []
LOGDIR: Path | None = None
CORE_LOCK_FH = None


def ts() -> str:
    return datetime.now().strftime("%H:%M:%S")


def stamp() -> str:
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def write_state(status: str, stage: str, message: str, **extra: Any) -> None:
    payload = {
        "status": status,
        "stage": stage,
        "message": message,
        "time": ts(),
        "timestamp": time.time(),
        "pid": os.getpid(),
        "logdir": str(LOGDIR) if LOGDIR else "-",
        **extra,
    }
    tmp = STATE_FILE.with_suffix(".tmp")
    tmp.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    tmp.replace(STATE_FILE)
    print(f"[{ts()}] {message}", flush=True)


def fail(stage: str, message: str) -> None:
    write_state("LOI", stage, message)
    raise RuntimeError(message)


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b""):
            h.update(chunk)
    return h.hexdigest()


def check_file(path: str, label: str, expected: str | None = None) -> None:
    p = Path(path)
    if not p.is_file():
        fail("KIEM_TRA_TEP", f"Thiếu tệp {label}: {p}")
    if expected:
        got = sha256(p)
        if got != expected:
            fail("KIEM_TRA_TEP", f"Sai mã kiểm tra {label}. Không tự thay đổi tệp ổn định.")


def run(cmd: list[str], timeout: float = 5.0, check: bool = False, env: dict[str, str] | None = None) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        cmd,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        timeout=timeout,
        check=check,
        env=env or ROS_ENV,
    )


def ros_cli(args: list[str], timeout: float = 4.0) -> tuple[int, str]:
    try:
        cp = run(["ros2", *args], timeout=timeout)
        return cp.returncode, (cp.stdout or "").strip()
    except subprocess.TimeoutExpired:
        return 124, "Hết thời gian chờ lệnh ROS2"
    except Exception as exc:
        return 125, f"{type(exc).__name__}: {exc}"


def wait_until(fn, timeout_s: float, interval: float = 0.4) -> bool:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        if STOP_REQUESTED.is_set() or EMERGENCY_REQUESTED.is_set():
            return False
        try:
            if fn():
                return True
        except Exception:
            pass
        time.sleep(interval)
    return False


def wait_node(name: str, timeout_s: float) -> None:
    def ok() -> bool:
        rc, out = ros_cli(["node", "list"], 3.0)
        return rc == 0 and name in out.splitlines()
    if not wait_until(ok, timeout_s):
        fail("KHOI_DONG", f"Nút ROS chưa sẵn sàng: {name}")
    print(f"[{ts()}] Nút ROS sẵn sàng: {name}", flush=True)


def wait_service(name: str, timeout_s: float) -> None:
    def ok() -> bool:
        rc, out = ros_cli(["service", "list"], 3.0)
        return rc == 0 and name in out.splitlines()
    if not wait_until(ok, timeout_s):
        fail("KHOI_DONG", f"Dịch vụ ROS chưa sẵn sàng: {name}")
    print(f"[{ts()}] Dịch vụ ROS sẵn sàng: {name}", flush=True)


def wait_lifecycle_active(name: str, timeout_s: float) -> None:
    per_call = float(TH.get("lifecycle_cli_timeout_s", 4))
    def ok() -> bool:
        rc, out = ros_cli(["lifecycle", "get", name], per_call)
        return rc == 0 and "active" in out.lower()
    if not wait_until(ok, timeout_s):
        fail("NAV2", f"{name} chưa chuyển sang trạng thái hoạt động")
    print(f"[{ts()}] Đang hoạt động: {name}", flush=True)


def wait_topic_sample(topic: str, timeout_s: float, sensor_qos: bool = False) -> None:
    per_call = float(TH.get("topic_cli_timeout_s", 4))
    def ok() -> bool:
        cmd = ["topic", "echo", topic]
        if sensor_qos:
            cmd += ["--qos-reliability", "best_effort", "--qos-durability", "volatile"]
        cmd += ["--once"]
        rc, _ = ros_cli(cmd, per_call)
        return rc == 0
    if not wait_until(ok, timeout_s, 0.3):
        fail("DU_LIEU_ROS", f"Chưa nhận được dữ liệu mới từ {topic}")
    print(f"[{ts()}] Dữ liệu đã có: {topic}", flush=True)


def param_get(node: str, key: str, timeout_s: float | None = None, attempts: int | None = None) -> str:
    """Đọc tham số ROS2 có retry hữu hạn, vẫn fail-closed nếu thật sự không đọc được."""
    per_call = float(timeout_s or TH.get("param_cli_timeout_s", 5))
    tries = int(attempts or TH.get("param_cli_attempts", 3))
    last = "chưa có phản hồi"
    for i in range(1, tries + 1):
        rc, out = ros_cli(["param", "get", node, key], per_call)
        if rc == 0 and out.strip():
            if i > 1:
                print(f"[{ts()}] Đã đọc được {node} / {key} ở lần {i}/{tries}", flush=True)
            return out
        last = out or f"mã trả về {rc}"
        if i < tries:
            print(f"[{ts()}] Đọc {node} / {key} chưa thành công (lần {i}/{tries}); đang thử lại...", flush=True)
            time.sleep(0.6 + 0.4 * i)
    fail("KIEM_TRA_CAU_HINH", f"Không đọc được {node} / {key} sau {tries} lần: {last}")
    raise AssertionError("unreachable")

def start_group(name: str, argv: list[str], logfile: Path, env: dict[str, str] | None = None) -> subprocess.Popen:
    f = logfile.open("ab", buffering=0)
    p = subprocess.Popen(argv, stdout=f, stderr=subprocess.STDOUT, start_new_session=True, env=env or ROS_ENV)
    PROCS.append({"name": name, "proc": p, "log": logfile, "file": f})
    time.sleep(0.7)
    if p.poll() is not None:
        try:
            tail = logfile.read_text(encoding="utf-8", errors="replace").splitlines()[-60:]
        except Exception:
            tail = []
        fail("KHOI_DONG", f"{name} dừng ngay khi khởi động. " + " | ".join(tail[-5:]))
    print(f"[{ts()}] Đã khởi động {name} PID={p.pid}", flush=True)
    return p


def stop_group(item: dict[str, Any], sig: signal.Signals = signal.SIGINT, wait_s: float = 6.0) -> None:
    p: subprocess.Popen = item["proc"]
    if p.poll() is not None:
        return
    try:
        os.killpg(p.pid, sig)
    except ProcessLookupError:
        return
    end = time.monotonic() + wait_s
    while time.monotonic() < end and p.poll() is None:
        time.sleep(0.2)
    if p.poll() is None:
        try:
            os.killpg(p.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass


def core_alive() -> bool:
    try:
        pid = int(PID_FILE.read_text().strip())
        os.kill(pid, 0)
        return True
    except Exception:
        return False


def supervisor_cmd(cmd: str, timeout_s: float = 1.0) -> tuple[bool, str]:
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout_s)
    try:
        s.sendto(cmd.encode(), ("127.0.0.1", 5005))
        data, _ = s.recvfrom(4096)
        return True, data.decode(errors="replace").strip()
    except Exception as exc:
        return False, f"{type(exc).__name__}: {exc}"
    finally:
        s.close()


def parse_status(text: str) -> dict[str, str]:
    out: dict[str, str] = {}
    if not text.startswith("STATUS "):
        return out
    for tok in text.split()[1:]:
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def wait_supervisor(timeout_s: float = 12.0) -> dict[str, str]:
    holder: dict[str, str] = {}
    def ok() -> bool:
        nonlocal holder
        good, text = supervisor_cmd("STATUS", 0.8)
        if not good:
            return False
        holder = parse_status(text)
        return bool(holder)
    if not wait_until(ok, timeout_s, 0.3):
        fail("GIAM_SAT", "Bộ giám sát nhiệm vụ chưa phản hồi")
    return holder


def wait_esp32_clear(timeout_s: float) -> dict[str, str]:
    holder: dict[str, str] = {}
    def ok() -> bool:
        nonlocal holder
        good, text = supervisor_cmd("STATUS", 0.8)
        if not good:
            return False
        holder = parse_status(text)
        return holder.get("esp32", "-") != "-" and holder.get("sensor1") == "CLEAR"
    if not wait_until(ok, timeout_s, 0.4):
        fail("ESP32", "ESP32 chưa kết nối hoặc Cảm biến 1 chưa ở trạng thái KHÔNG CÓ VẬT")
    return holder


def wait_camera(timeout_s: float = 15.0) -> None:
    status = Path("/tmp/amr_b2_camera_broker/status.json")
    frame = Path("/tmp/amr_b2_camera_broker/latest.jpg")
    def ok() -> bool:
        try:
            d = json.loads(status.read_text(encoding="utf-8"))
            age = time.time() - float(d.get("timestamp") or 0)
            return d.get("state") == "LIVE" and age < 1.5 and frame.is_file()
        except Exception:
            return False
    if not wait_until(ok, timeout_s, 0.25):
        fail("CAMERA", "Camera chưa có hình ảnh trực tiếp")


def hardware_check() -> None:
    for key, p in HW.items():
        if not Path(p).exists():
            fail("PHAN_CUNG", f"Không tìm thấy {key}: {p}")
    print(f"[{ts()}] Phần cứng USB/camera đã được nhận diện", flush=True)


def hash_check() -> None:
    check_file(STABLE["t1"], "T1", HASHES.get("t1"))
    check_file(STABLE["t2"], "T2", HASHES.get("t2"))
    check_file(STABLE["t3"], "T3", HASHES.get("t3"))
    check_file(STABLE["t4"], "T4", HASHES.get("t4"))
    check_file(STABLE["home_localizer"], "Home Localizer")
    check_file(STABLE["mission"], "Mission B2", HASHES.get("mission"))
    check_file(STABLE["stations"], "Stations", HASHES.get("stations"))
    check_file(STABLE["map"], "Map FIX12", HASHES.get("map_yaml"))


def duplicate_check() -> None:
    patterns = [
        "start_amr_system_v5_B2_FINAL_RC1",
        "amr_terminal1_ekf_V2_2A_TEST.sh",
        "t2_mppi_035_FIX7_MAP_FIX12.sh",
        "turn_safety_gate_v3_FIX7.py",
        "collision_monitor_node.launch.py",
        "amr_mission_supervisor_v5_b2_workmode.py",
        "HT_V1_SUP_AMR.py",
        "HT_V1_CAM_AMR.py",
        "HT_V1_CORE_AMR.py start",
    ]
    try:
        out = run(["pgrep", "-af", "|".join(patterns)], 2.0).stdout or ""
    except Exception:
        out = ""
    lines = [x for x in out.splitlines() if str(os.getpid()) not in x and "pgrep -af" not in x]
    if lines:
        fail("TRUNG_TIEN_TRINH", "Phát hiện tiến trình AMR cũ còn chạy: " + " || ".join(lines[:8]))


def stationary_guard(sample_s: float = 2.2, first_sample_wait_s: float | None = None) -> tuple[bool, float | None, float | None, int]:
    """Chờ mẫu /odom đầu tiên riêng, rồi mới lấy cửa sổ mẫu để đánh giá đứng yên."""
    first_wait = float(first_sample_wait_s or TH.get("odom_first_sample_wait_s", 4.0))
    code = r"""
import json,sys,time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
class N(Node):
  def __init__(self):
    super().__init__('ht_v1_stationary_guard')
    self.val=[]
    qos=QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=20,
                   reliability=ReliabilityPolicy.BEST_EFFORT,
                   durability=DurabilityPolicy.VOLATILE)
    self.create_subscription(Odometry,'/odom',self.cb,qos)
  def cb(self,m):
    self.val.append((abs(float(m.twist.twist.linear.x)),abs(float(m.twist.twist.angular.z))))
rclpy.init(); n=N(); first=float(sys.argv[1]); sample=float(sys.argv[2])
first_end=time.monotonic()+first
while time.monotonic()<first_end and not n.val:
  rclpy.spin_once(n,timeout_sec=0.10)
if n.val:
  n.val=[]
  end=time.monotonic()+sample
  while time.monotonic()<end:
    rclpy.spin_once(n,timeout_sec=0.08)
vals=n.val; n.destroy_node(); rclpy.shutdown()
print(json.dumps({'n':len(vals),'vmax':max([v for v,w in vals],default=None),'wmax':max([w for v,w in vals],default=None)}))
"""
    try:
        cp = run(["/usr/bin/python3", "-c", code, str(first_wait), str(sample_s)],
                 timeout=first_wait + sample_s + 5, env=ROS_ENV)
        lines = [x for x in (cp.stdout or "").splitlines() if x.strip()]
        d = json.loads(lines[-1] if lines else "{}")
        n = int(d.get("n", 0))
        vmax = None if d.get("vmax") is None else float(d["vmax"])
        wmax = None if d.get("wmax") is None else float(d["wmax"])
        if n < int(TH.get("odom_min_samples", 5)) or vmax is None or wmax is None:
            return False, vmax, wmax, n
        ok = vmax <= float(TH["dung_yen_v_mps"]) and wmax <= float(TH["dung_yen_w_rad_s"])
        return ok, vmax, wmax, n
    except Exception:
        return False, None, None, 0

def reset_a_h() -> None:
    rc, out = ros_cli(["service", "call", "/amr/reset_to_A_H", "std_srvs/srv/Trigger", "{}"], 12.0)
    norm = out.replace(" ", "").lower()
    if rc != 0 or not ("success=true" in norm or "success:true" in norm):
        fail("DINH_VI_A_H", f"Không đặt lại định vị A_H thành công: {out}")
    print(f"[{ts()}] Đã đặt lại định vị tại A_H", flush=True)


def mission_preflight(logfile: Path) -> None:
    """Chạy preflight B2 qua wrapper retry ROS2 param; tuyệt đối không gửi motion goal."""
    env = ROS_ENV.copy()
    env["HT_V1_MISSION_GOC"] = STABLE["mission"]
    wrapper = BASE / "HT_V1_B2_AMR.py"
    check_file(str(wrapper), "Bộ chạy nhiệm vụ HT_V1.1")
    try:
        cp = subprocess.run(
            ["/usr/bin/python3", str(wrapper), "--preflight-only"],
            cwd=str(BASE),
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=float(TH.get("preflight_timeout_s", 120)),
            env=env,
        )
        logfile.write_text(cp.stdout or "", encoding="utf-8")
    except subprocess.TimeoutExpired as exc:
        out = exc.stdout if isinstance(exc.stdout, str) else ""
        logfile.write_text(out or "", encoding="utf-8")
        fail("TU_KIEM_TRA", "Tự kiểm tra B2 vượt quá thời gian cho phép")

    text = logfile.read_text(encoding="utf-8", errors="replace")
    if cp.returncode == 0 and "PREFLIGHT-ONLY PASS: NO MOTION GOAL WAS SENT" in text:
        print(f"[{ts()}] Tự kiểm tra tuyến B2: ĐẠT", flush=True)
        return

    important = []
    for line in text.splitlines():
        u = line.upper()
        if ("FAIL PREFLIGHT" in u or "MISSION STOPPED AT STAGE" in u or
                "SAFE STOP" in u or "ERROR" in u or "TIMEOUT" in u):
            important.append(line.strip())
    detail = " | ".join(important[-5:]) if important else "Không tìm thấy dòng nguyên nhân trong nhật ký"
    fail("TU_KIEM_TRA", f"Tự kiểm tra B2 không đạt (mã={cp.returncode}): {detail}")

def fast_zero_velocity() -> None:
    msg = "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    for _ in range(2):
        try:
            ros_cli(["topic", "pub", "--once", "/cmd_vel_nav", "geometry_msgs/msg/Twist", msg], 1.2)
        except Exception:
            pass


def emergency_stop() -> None:
    write_state("DUNG_KHAN", "DUNG_KHAN", "Đã nhận lệnh DỪNG KHẨN - đang dừng chuyển động")
    supervisor_cmd("EMERGENCY_STOP", 0.5)
    fast_zero_velocity()
    try:
        subprocess.Popen(["/usr/bin/python3", str(BASE / "HT_V1_ACT_AMR.py"), "stop"], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    except Exception:
        pass


def cleanup(reason: str, final_status: str = "DA_DUNG") -> None:
    try:
        supervisor_cmd("WORK_OFF", 0.4)
    except Exception:
        pass
    if EMERGENCY_REQUESTED.is_set():
        emergency_stop()
    ok, vmax, wmax, n = stationary_guard(2.0)
    if n == 0 or vmax is None or wmax is None:
        print(f"[{ts()}] Chưa nhận được /odom khi kiểm tra dừng; runtime đang được dọn sau lệnh dừng/fail-safe", flush=True)
    else:
        print(f"[{ts()}] Kiểm tra đứng yên trước khi tháo tầng an toàn: n={n} vmax={vmax:.3f} wmax={wmax:.3f} dat={ok}", flush=True)
    for item in reversed(PROCS):
        stop_group(item)
    for item in PROCS:
        try:
            item["file"].close()
        except Exception:
            pass
    PROCS.clear()
    try:
        PID_FILE.unlink(missing_ok=True)
    except Exception:
        pass
    if final_status == "LOI":
        write_state("LOI", "LOI", f"Hệ thống đã dừng do lỗi: {reason}")
    elif final_status == "DUNG_KHAN":
        write_state("DUNG_KHAN", "DUNG_KHAN", "Hệ thống đã dừng khẩn và giữ trạng thái cần khôi phục")
    else:
        write_state("DA_DUNG", "DA_DUNG", f"Hệ thống đã dừng: {reason}")


def signal_handler(signum, _frame):
    if signum == signal.SIGUSR1:
        EMERGENCY_REQUESTED.set()
        STOP_REQUESTED.set()
    else:
        STOP_REQUESTED.set()


def acquire_lock() -> None:
    global CORE_LOCK_FH
    CORE_LOCK_FH = LOCK_FILE.open("a+")
    try:
        fcntl.flock(CORE_LOCK_FH.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print("Hệ thống AMR đã có một tiến trình điều phối đang chạy.", file=sys.stderr)
        raise SystemExit(11)
    PID_FILE.write_text(str(os.getpid()), encoding="utf-8")


def start_sequence() -> None:
    global LOGDIR
    signal.signal(signal.SIGTERM, signal_handler)
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGUSR1, signal_handler)
    acquire_lock()
    LOGDIR = HOME / "AMR_LOGS" / stamp()
    LOGDIR.mkdir(parents=True, exist_ok=True)
    write_state("DANG_KHOI_DONG", "BAT_DAU", "Đang khởi động hệ thống AMR tuyến B2")

    try:
        hash_check()
        hardware_check()
        duplicate_check()

        # Supervisor mở sớm để ESP32 được ACK ngay. Work Mode vẫn khóa cho tới READY.
        write_state("DANG_KHOI_DONG", "ESP32", "Đang mở kết nối ESP32 và bộ giám sát nhiệm vụ")
        sup_env = ROS_ENV.copy()
        sup_env["AMR_B2_WORK_AUTO"] = "0"
        sup_env["AMR_B2_MISSION_FILE"] = str(BASE / "HT_V1_B2_AMR.py")
        sup_env["HT_V1_MISSION_GOC"] = STABLE["mission"]
        sup_env["AMR_CAMERA_RUNNER"] = str(BASE / "HT_V1_CAM_RUN_AMR.py")
        sup_env["HT_V1_LOG_DIR"] = str(LOGDIR)
        start_group("SUPERVISOR", ["/usr/bin/python3", "-u", str(BASE / "HT_V1_SUP_AMR.py")], LOGDIR / "SUPERVISOR.log", sup_env)
        wait_supervisor(10)
        good, sup_text = supervisor_cmd("STATUS", 0.8)
        if good:
            sup_now = parse_status(sup_text)
            if sup_now.get("esp32", "-") != "-":
                print(f"[{ts()}] ESP32 đã bắt tay sớm: {sup_now.get('esp32')}", flush=True)
            else:
                print(f"[{ts()}] Bộ giám sát đã sẵn sàng; đang chờ ESP32 tự bắt tay trong nền", flush=True)

        write_state("DANG_KHOI_DONG", "T1", "Đang khởi động điều khiển nền, odom, IMU và LiDAR")
        start_group("T1", ["bash", STABLE["t1"]], LOGDIR / "T1.log")
        wait_node("/amr_stm32_bridge", 20)
        wait_node("/rplidar_node", 20)
        wait_node("/ekf_filter_node", 20)
        wait_topic_sample("/odom_raw", 15)
        wait_topic_sample("/imu/data", 15)
        wait_topic_sample("/odom", 15)
        wait_topic_sample("/scan", 20, sensor_qos=True)
        time.sleep(float(TH.get("startup_settle_s", 10)))
        ok, vmax, wmax, n = stationary_guard(2.2)
        if n == 0 or vmax is None or wmax is None:
            fail("DUNG_YEN", "Không nhận được dữ liệu /odom để xác nhận robot đứng yên")
        if not ok:
            fail("DUNG_YEN", f"Robot chưa đứng yên: vmax={vmax:.3f} m/s, wmax={wmax:.3f} rad/s, mẫu={n}")

        write_state("DANG_KHOI_DONG", "DINH_VI_A_H", "Đang chuẩn bị định vị tại A_H")
        start_group("HOME_LOCALIZER", ["/usr/bin/python3", STABLE["home_localizer"]], LOGDIR / "HOME_LOCALIZER.log")
        wait_node("/amr_home_localizer", 15)
        wait_service("/amr/reset_to_A_H", 15)

        write_state("DANG_KHOI_DONG", "NAV2", "Đang khởi động bản đồ và Navigation2")
        start_group("T2", ["bash", STABLE["t2"]], LOGDIR / "T2.log")
        wait_node("/amr_scan_midstamp_v1_0", 18)
        for node in ["/map_server", "/amcl", "/controller_server", "/planner_server", "/bt_navigator", "/velocity_smoother"]:
            wait_lifecycle_active(node, 30)
        wait_topic_sample("/scan_amcl_mid", 15, sensor_qos=True)
        print(f"[{ts()}] Đang ổn định ROS graph trước khi kiểm tra cấu hình Navigation2", flush=True)
        time.sleep(float(TH.get("param_audit_settle_s", 2.0)))

        map_now = param_get("/map_server", "yaml_filename")
        if STABLE["map"] not in map_now:
            fail("BAN_DO", f"Bản đồ đang chạy không phải FIX12: {map_now}")
        plugin = param_get("/controller_server", "FollowPath.plugin")
        if "nav2_mppi_controller::MPPIController" not in plugin:
            fail("DIEU_KHIEN", f"Bộ điều khiển FollowPath không phải MPPI: {plugin}")
        vx = param_get("/controller_server", "FollowPath.vx_max")
        if "0.35" not in vx:
            fail("DIEU_KHIEN", f"Giới hạn vận tốc MPPI không đúng 0.35 m/s: {vx}")
        reset_a_h()

        write_state("DANG_KHOI_DONG", "AN_TOAN_1", "Đang khởi động cổng an toàn vận tốc")
        start_group("T3", ["bash", STABLE["t3"]], LOGDIR / "T3.log")
        wait_node("/amr_turn_safety_gate_v3_dynamic", 18)

        write_state("DANG_KHOI_DONG", "AN_TOAN_2", "Đang khởi động giám sát va chạm")
        start_group("T4", ["bash", STABLE["t4"]], LOGDIR / "T4.log")
        wait_lifecycle_active("/collision_monitor", 25)

        write_state("DANG_KHOI_DONG", "TU_KIEM_TRA", "Đang tự kiểm tra tuyến B2 - không gửi lệnh chuyển động")
        mission_preflight(LOGDIR / "TU_KIEM_TRA_B2.log")
        ok, vmax, wmax, n = stationary_guard(2.0)
        if n == 0 or vmax is None or wmax is None:
            fail("TU_KIEM_TRA", "Không nhận được /odom để xác nhận robot đứng yên sau tự kiểm tra")
        if not ok:
            fail("TU_KIEM_TRA", f"Robot không đứng yên sau tự kiểm tra: vmax={vmax:.3f}, wmax={wmax:.3f}, mẫu={n}")

        write_state("DANG_KHOI_DONG", "CAMERA", "Đang khởi động camera dùng chung cho giám sát và QR")
        try:
            import shutil
            shutil.rmtree("/tmp/amr_b2_camera_broker", ignore_errors=True)
        except Exception:
            pass
        start_group("CAMERA", ["/usr/bin/python3", "-u", str(BASE / "HT_V1_CAM_AMR.py")], LOGDIR / "CAMERA.log")
        wait_camera(15)

        write_state("DANG_KHOI_DONG", "ESP32_SAN_SANG", "Đang xác nhận ESP32 và Cảm biến 1 sẵn sàng")
        wait_supervisor(5)
        wait_esp32_clear(float(CFG["he_thong"].get("esp32_timeout_s", 30)))
        good, reply = supervisor_cmd("WORK_ON", 1.0)
        if not good or "OK" not in reply:
            fail("CHE_DO_LAM_VIEC", f"Không bật được chế độ nhận nhiệm vụ B2: {reply}")
        st = wait_supervisor(5)
        if st.get("work") != "1":
            fail("CHE_DO_LAM_VIEC", "Bộ giám sát chưa xác nhận chế độ làm việc")

        write_state("SAN_SANG", "SAN_SANG", "Hệ thống đã sẵn sàng nhận nhiệm vụ B2")
        while not STOP_REQUESTED.wait(0.2):
            for item in PROCS:
                p: subprocess.Popen = item["proc"]
                if p.poll() is not None:
                    fail("MAT_TIEN_TRINH", f"{item['name']} đã dừng ngoài dự kiến")
            # Keep state fresh without changing operator-facing stage.
            try:
                st = wait_supervisor(1.2)
                if st.get("state") == "BUSY":
                    write_state("DANG_THUC_HIEN", "NHIEM_VU_B2", "Đang thực hiện nhiệm vụ B2", supervisor=st)
                elif st.get("work") == "1":
                    write_state("SAN_SANG", "SAN_SANG", "Hệ thống đã sẵn sàng nhận nhiệm vụ B2", supervisor=st)
                else:
                    write_state("CAN_KHOI_PHUC", "CAN_KHOI_PHUC", "Đã tạm dừng nhận nhiệm vụ; khôi phục cơ cấu nếu cần rồi nhấn TIẾP TỤC", supervisor=st)
            except Exception:
                pass

        cleanup("Dừng khẩn" if EMERGENCY_REQUESTED.is_set() else "Người vận hành yêu cầu dừng", "DUNG_KHAN" if EMERGENCY_REQUESTED.is_set() else "DA_DUNG")
    except Exception as exc:
        try:
            if not STOP_REQUESTED.is_set():
                write_state("LOI", "LOI", f"Khởi động/vận hành thất bại: {exc}")
            cleanup(str(exc), "LOI")
        except Exception:
            pass
        raise


def stop_core(emergency: bool = False) -> int:
    if not PID_FILE.exists():
        print("Hệ thống AMR chưa chạy.")
        return 0
    try:
        pid = int(PID_FILE.read_text().strip())
        os.kill(pid, signal.SIGUSR1 if emergency else signal.SIGTERM)
        print("Đã gửi yêu cầu DỪNG KHẨN." if emergency else "Đã gửi yêu cầu dừng hệ thống.")
        return 0
    except ProcessLookupError:
        PID_FILE.unlink(missing_ok=True)
        print("Tiến trình cũ không còn hoạt động; đã dọn PID.")
        return 0
    except Exception as exc:
        print(f"Không gửi được yêu cầu dừng: {exc}", file=sys.stderr)
        return 2


def status_core() -> int:
    try:
        print(STATE_FILE.read_text(encoding="utf-8"))
    except Exception:
        print(json.dumps({"status": "DA_TAT", "stage": "DA_TAT", "message": "Hệ thống chưa khởi động"}, ensure_ascii=False))
    return 0


def main() -> int:
    cmd = sys.argv[1] if len(sys.argv) > 1 else "status"
    if cmd == "start":
        start_sequence()
        return 0
    if cmd == "stop":
        return stop_core(False)
    if cmd == "emergency":
        return stop_core(True)
    if cmd == "status":
        return status_core()
    print("Dùng: HT_V1_CORE_AMR.py start|stop|emergency|status", file=sys.stderr)
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
