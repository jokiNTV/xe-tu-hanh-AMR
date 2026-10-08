#!/usr/bin/env python3
"""
AMR DEMO FAST V3 FIX11 B2 QR
=====================

Purpose:
    Fast, practical acceptance runner for the current AMR project.
    It deliberately prioritizes stable route execution + final yaw over
    centimeter-level XY docking.

Mission:
    A_H (start check)
      -> Nav2 B1 or B2 using Theta* + MPPI
         (coarse handoff XY<=15 cm, |yaw|<=7.5 deg)
      -> one-shot yaw align at pickup station (<3 deg)
      -> DriveOnHeading into pickup zone
      -> simulated PICK hold (actuator hook not integrated yet)
      -> explicit mission BackUp 2.00 m after PICK (commissioning value)
      -> Nav2 TH
      -> one-shot yaw align at TH (<3 deg)
      -> TH +0.40 m ODOM HEADING-LOCK -> DROP -> BackUp 0.40 m to TH -> CLOSE empty clamp
      -> return load-state bookkeeping to EMPTY after retreat (geometry unchanged in FIX7 TEST)
      -> Nav2 A_H
      -> one-shot yaw align at A_H (<3 deg)
      -> SAFE STOP / DONE

Locked runtime architecture:
    Theta* -> MPPI -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
    -> Turn Safety Gate V3 Dynamic -> /cmd_vel_guarded -> Collision Monitor
    -> /cmd_vel_safe -> amr_stm32_bridge -> STM32.

Important:
    - MPPI remains the active controller. DWB is NOT used.
    - Recovery BT has NO automatic BackUp and NO automatic Spin.
      Mission-level blocked/status-6 recovery is bounded to one BackUp/rescan/replan attempt.
    - The BackUp after pickup is an EXPLICIT 2.00 m mission step by default, not recovery.
    - The proven first turn_relative correction is retained. YAW_RETRY1 adds
      exactly one bounded fine correction only when the first result remains
      outside 3 deg but inside the existing 8 deg correction envelope.
    - Every correction keeps the existing LiDAR swept-area check.
    - B2 default pickup distance is 1.50 m (FIX7 commissioning target; verify physically).
    - B1 pickup distance has NO invented default. Pass --pickup-distance after measurement.
    - FIX7 TEST deliberately models the same body footprint before/after PICK.
      Real carried-load protrusion is not represented; keep the test area physically clear.
    - This TEST file does not modify any stable baseline file.

Status at creation:
    FIX10 is the direct baseline. FIX11 changes only the TH drop geometry:
    (1) approach: +0.50 m -> +0.40 m;
    (2) retreat:  -0.50 m -> -0.40 m.
    FIX10 odom yaw-hold, safe STM32 #2 discovery, QR gate, B2 navigation,
    PICK / DROP / CLOSE / return route are retained unchanged.
"""

import argparse
import fcntl
import json
import math
import re
import os
import signal
import statistics
import subprocess
import sys
import tempfile
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
import yaml
import xml.etree.ElementTree as ET
try:
    import serial
except Exception:
    serial = None
try:
    import cv2
except Exception:
    cv2 = None
try:
    from pyzbar.pyzbar import decode as zbar_decode
except Exception:
    zbar_decode = None
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from geometry_msgs.msg import Point32, Polygon, Twist, Vector3Stamped
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import BackUp, DriveOnHeading, NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
try:
    from rclpy.signals import SignalHandlerOptions
except Exception:
    SignalHandlerOptions = None
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener


# =============================================================================
# LOCKED PATHS / TOPICS
# =============================================================================

DEFAULT_STATIONS = str(
    Path.home()
    / "amr_ws/amr_new_nav/mission_v1_clean/stations_v1_clean.yaml"
)

NAV_ACTION = "/navigate_to_pose"
DRIVE_ACTION = "/drive_on_heading"
BACKUP_ACTION = "/backup"

CMD_NAV = "/cmd_vel_nav"
CMD_SMOOTH = "/cmd_vel"
CMD_GUARDED = "/cmd_vel_guarded"
CMD_SAFE = "/cmd_vel_safe"

TURN_TOPIC = "/amr/drive/turn_relative"
HEADING_STATE_TOPIC = "/amr/drive/heading_state"
HEADING_RATE_TOPIC = "/amr/drive/heading_rate"
ODOM_TOPIC = "/odom"
SCAN_TOPIC = "/scan"

MAP_FRAME = "map"
BASE_FRAME = "base_link"

# FIX9 B2 route is intentionally compiled into this standalone file.
FIX8_ROUTE = "B2"  # retained variable name to minimize changes from stable FIX8
FIX8_PICKUP_DISTANCE_M = 1.50
FIX8_PICK_EXIT_BACKUP_M = 2.00

# TEST-ONLY physical footprint policy. Costmap padding remains 0.05 m in Nav2.
# For the current controlled test, keep the SAME footprint before and after PICK.
# The real carried load may protrude beyond this model; the test area must be kept clear.
FOOTPRINT_PADDING_M = 0.05

EMPTY_FRONT_M = 0.943
EMPTY_REAR_M = 0.455
EMPTY_LEFT_M = 0.370
EMPTY_RIGHT_M = 0.370

LOADED_FRONT_M = 0.943
LOADED_REAR_M = 0.455
LOADED_LEFT_M = 0.370
LOADED_RIGHT_M = 0.370

LOCAL_FOOTPRINT_TOPIC = "/local_costmap/footprint"
GLOBAL_FOOTPRINT_TOPIC = "/global_costmap/footprint"
LOAD_STATE_TOPIC = "/amr/load_state"

# FIX7 TEST hard-stop envelope follows the unchanged body footprint.
# Padded body is approximately front=+0.993, rear=-0.505, side=+/-0.420 m.
# Keep the previously stable last-resort hard-stop envelope around that body.
CM_HARDSTOP_POINTS = [
    1.030, 0.460,
    1.030, -0.460,
    -0.545, -0.460,
    -0.545, 0.460,
]
CM_APPROACH_TTC_S = 0.60
CM_FOOTPRINT_TOPIC = "/local_costmap/published_footprint"

# Demo policies: coarse XY, priority on stable final yaw.
YAW_RELEASE_DEG = 3.0
YAW_MAX_CORRECTION_DEG = 8.0

# YAW_RETRY1 hotfix:
# Keep the proven first final-yaw correction unchanged. If it finishes just
# outside the 3 deg release band, allow exactly ONE bounded correction after
# re-reading map pose and re-running the swept-area safety check. The retry
# rate is 0.15 rad/s (already used by the stable A_H alignment) to give the
# loaded robot a little more authority against static friction.
YAW_FINE_RETRY_MAX = 1
YAW_FINE_RETRY_RATE_RAD_S = 0.15

NAV_POSITION_HANDOFF_M = 0.15
STATION_HANDOFF_XY_M = {
    "B1": 0.15,
    "B2": 0.15,
    "TH": 0.15,
    "A_H": 0.15,
}
NAV_YAW_HANDOFF_DEG = 7.5
NAV_HANDOFF_STABLE_S = 0.25

# Anti-hunt remains fail-safe. At 15 cm handoff this guard arms at 25 cm.
TERMINAL_GUARD_EXTRA_M = 0.10
TERMINAL_GUARD_MAX_S = 6.0
TURN_RATE_DEFAULT_RAD_S = 0.12
TURN_RATE_A_H_RAD_S = 0.15
TURN_EXTRA_TIMEOUT_S = 2.0
TURN_SWEEP_MARGIN_DEG = 2.0
TURN_SWEEP_STEP_DEG = 1.0

START_HOME_MAX_XY_M = 0.30

# FIX7 mission motion speeds. Navigation cruise ceiling is set in mppi_040_FIX7.yaml.
# Pickup stays deliberate; reverse is raised to 0.15 m/s as the agreed first test.
DEFAULT_B2_PICKUP_DISTANCE_M = 1.50
DEFAULT_PICK_EXIT_BACKUP_M = 2.00
MAX_PICK_EXIT_BACKUP_M = 2.00
DEFAULT_PICKUP_SPEED_MPS = 0.08
DEFAULT_BACKUP_SPEED_MPS = 0.15
MAX_PICKUP_SPEED_MPS = 0.10
MAX_BACKUP_SPEED_MPS = 0.15
PICK_HOLD_S = 1.0
DROP_HOLD_S = 1.0

# TH drop approach requested for V3:
# align at TH -> +0.30 m -> DROP -> -0.30 m back to TH -> then navigate A_H.
TH_DROP_FORWARD_M = 0.40
TH_DROP_FORWARD_SPEED_MPS = 0.10
TH_DROP_BACKUP_M = 0.40
TH_DROP_BACKUP_SPEED_MPS = 0.15

# FIX11: only TH drop geometry changes to +0.40 m / -0.40 m; FIX10 yaw-hold is retained.
# Hold the robot's actual yaw at the start of this short move using fresh /odom,
# while publishing through /cmd_vel_nav so the normal safety chain stays intact.
TH_LOCK_K_YAW = 1.80
TH_LOCK_MAX_W_RAD_S = 0.10
TH_LOCK_MIN_V_MPS = 0.035
TH_LOCK_SLOWDOWN_M = 0.15
TH_LOCK_CONTROL_HZ = 20.0
TH_LOCK_ODOM_MAX_AGE_S = 0.30
TH_LOCK_SCAN_MAX_AGE_S = 0.40
TH_LOCK_ABORT_YAW_DEG = 12.0
TH_LOCK_ABORT_LAT_M = 0.12
TH_LOCK_STALL_S = 2.5

# Bounded mission-level escape recovery.
# It now handles BOTH near-goal anti-hunt and a general route-blocked condition.
# Sequence: cancel/stop -> BackUp -> clear costmaps -> allow scans to remark -> replan.
ESCAPE_BACKUP_M = 0.25
ESCAPE_BACKUP_SPEED_MPS = 0.15
ESCAPE_MAX_RETRIES = 0  # 0 = persistent/unlimited recoverable navigation cycles
RECOVERY_STOP_HOLD_S = 0.60
RECOVERY_BLOCKED_WAIT_S = 2.0
MOTION_RETRY_MAX_ATTEMPTS = 0  # 0 = persistent while sensors remain healthy
ROUTE_BLOCKED_MAX_S = 6.0  # detect blockage; FIX7 recovers instead of ending mission
ROUTE_BLOCKED_V_MAX_MPS = 0.025
ROUTE_BLOCKED_W_MAX_RAD_S = 0.035
ROUTE_BLOCKED_ODOM_FRESH_S = 0.30
ROUTE_BLOCKED_CMD_FRESH_S = 0.30
ROUTE_BLOCKED_START_GRACE_S = 2.0
ROUTE_PROGRESS_RESET_M = 0.05
ROUTE_PROGRESS_RESET_YAW_DEG = 5.0

# Recovery BT: clear costmaps + wait only. No automatic motion recovery.
RECOVERY_RETRIES = 6

# Required runtime nodes
REQUIRED_NODES = {
    "/controller_server",
    "/behavior_server",
    "/bt_navigator",
    "/planner_server",
    "/velocity_smoother",
    "/collision_monitor",
    "/amr_turn_safety_gate_v3_dynamic",
    "/amr_stm32_bridge",
}

# Topics to bag if they currently exist
BAG_TOPIC_CANDIDATES = [
    "/tf",
    "/tf_static",
    "/scan",
    "/cmd_vel_nav",
    "/cmd_vel",
    "/cmd_vel_guarded",
    "/cmd_vel_safe",
    "/odom",
    "/odom_raw",
    "/amcl_pose",
    "/goal_pose",
    "/amr/turn_safety_gate/status",
    "/amr/drive/heading_state",
    "/amr/drive/heading_rate",
    "/amr/drive/measured_wheel_rad_s",
    "/amr/drive/target_wheel_rad_s",
    "/amr/drive/pwm",
]


RECOVERY_BT_XML = f"""<!--
  GENERATED BY AMR_FULL_MISSION_RECOVERY_V2_1_SAFE2.py
  SAFE RECOVERY BT.
  IMPORTANT:
    - NO automatic BackUp inside Nav2 recovery.
    - NO automatic Spin.
    - Recovery may clear costmaps and wait only.
    - If it cannot recover, the mission aborts and SAFE STOP cancels all actions.
-->
<root main_tree_to_execute="MainTree">
  <BehaviorTree ID="MainTree">

    <RecoveryNode number_of_retries="{RECOVERY_RETRIES}" name="NavigateRecovery">

      <PipelineSequence name="NavigateWithReplanning">

        <RateController hz="2.0">
          <RecoveryNode number_of_retries="1" name="ComputePathToPose">
            <ComputePathToPose
              goal="{{goal}}"
              path="{{path}}"
              planner_id="GridBased"/>

            <ClearEntireCostmap
              name="ClearGlobalCostmap-Context"
              service_name="global_costmap/clear_entirely_global_costmap"/>
          </RecoveryNode>
        </RateController>

        <RecoveryNode number_of_retries="1" name="FollowPath">
          <FollowPath
            path="{{path}}"
            controller_id="FollowPath"/>

          <ClearEntireCostmap
            name="ClearLocalCostmap-Context"
            service_name="local_costmap/clear_entirely_local_costmap"/>
        </RecoveryNode>

      </PipelineSequence>

      <ReactiveFallback name="RecoveryFallback">
        <GoalUpdated/>

        <RoundRobin name="SafeRecoveryActions">

          <Sequence name="ClearingActions">
            <ClearEntireCostmap
              name="ClearLocalCostmap-Recovery"
              service_name="local_costmap/clear_entirely_local_costmap"/>

            <ClearEntireCostmap
              name="ClearGlobalCostmap-Recovery"
              service_name="global_costmap/clear_entirely_global_costmap"/>
          </Sequence>

          <Wait
            name="WaitRecovery"
            wait_duration="1"/>

        </RoundRobin>
      </ReactiveFallback>

    </RecoveryNode>
  </BehaviorTree>
</root>
"""



# =============================================================================
# MATH
# =============================================================================

def wrap_pi(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quat(q) -> float:
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def circular_mean(values: List[float]) -> float:
    if not values:
        raise ValueError("empty circular mean")
    s = sum(math.sin(v) for v in values)
    c = sum(math.cos(v) for v in values)
    return math.atan2(s, c)


def pose_error(
    pose: Tuple[float, float, float],
    target: Dict[str, float],
) -> Tuple[float, float]:
    x, y, yaw = pose
    xy = math.hypot(x - float(target["x"]), y - float(target["y"]))
    target_yaw = math.radians(float(target["yaw_deg"]))
    yaw_err = wrap_pi(target_yaw - yaw)
    return xy, yaw_err



# =============================================================================
# B2 QR AUTHENTICATION GATE - FIX9
# Camera is AUTHENTICATION ONLY. It does NOT control yaw/steering/position.
# =============================================================================

QR_CAMERA_BY_ID = "/dev/v4l/by-id/usb-XIFT_Web_Camera_SN00010-video-index0"
QR_EXPECTED_B2 = "BOX_001"
QR_WIDTH = 1280
QR_HEIGHT = 720
QR_FPS = 30
QR_TIMEOUT_S = 8.0
QR_WINDOW_S = 2.0
QR_REQUIRED_HITS = 5
QR_MIN_HIT_INTERVAL_S = 0.15


def qr_camera_preflight() -> Dict:
    if cv2 is None:
        raise RuntimeError("python3-opencv/cv2 is not available")
    if zbar_decode is None:
        raise RuntimeError("pyzbar/ZBar is not available; install libzbar0 python3-pyzbar")
    if not Path(QR_CAMERA_BY_ID).exists():
        raise RuntimeError(f"QR camera by-id path not found: {QR_CAMERA_BY_ID}")

    cam = os.path.realpath(QR_CAMERA_BY_ID)
    cap = cv2.VideoCapture(cam, cv2.CAP_V4L2)
    try:
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, QR_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, QR_HEIGHT)
        cap.set(cv2.CAP_PROP_FPS, QR_FPS)
        if not cap.isOpened():
            raise RuntimeError(f"cannot open QR camera: {cam}")
        ok, frame = cap.read()
        if not ok or frame is None:
            raise RuntimeError("QR camera opened but frame read failed")
        h, w = frame.shape[:2]
        return {
            "result": "PASS",
            "by_id": QR_CAMERA_BY_ID,
            "resolved_device": cam,
            "frame_width": int(w),
            "frame_height": int(h),
            "expected_qr": QR_EXPECTED_B2,
        }
    finally:
        cap.release()


def scan_b2_qr_gate(timeout_s: float = QR_TIMEOUT_S) -> Dict:
    """Return QR_VALID, QR_INVALID, or QR_NOT_FOUND. No robot motion is commanded."""
    if cv2 is None or zbar_decode is None:
        raise RuntimeError("QR dependencies unavailable")

    cam = os.path.realpath(QR_CAMERA_BY_ID)
    cap = cv2.VideoCapture(cam, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, QR_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, QR_HEIGHT)
    cap.set(cv2.CAP_PROP_FPS, QR_FPS)
    if not cap.isOpened():
        raise RuntimeError(f"cannot open QR camera: {cam}")

    histories = {}
    last_hit_at = {}
    t0 = time.monotonic()
    transcript = []
    try:
        while time.monotonic() - t0 < timeout_s:
            ok, frame = cap.read()
            if not ok or frame is None:
                continue

            h, w = frame.shape[:2]
            x0, x1 = int(w * 0.25), int(w * 0.80)
            y0, y1 = int(h * 0.30), int(h * 0.90)
            roi = frame[y0:y1, x0:x1]
            big = cv2.resize(roi, None, fx=2.0, fy=2.0, interpolation=cv2.INTER_CUBIC)
            found = zbar_decode(big)
            now = time.monotonic()

            payloads = []
            for obj in found:
                try:
                    data = obj.data.decode("utf-8").strip()
                except Exception:
                    continue
                if data and data not in payloads:
                    payloads.append(data)

            for data in payloads:
                hist = histories.setdefault(data, [])
                hist[:] = [t for t in hist if now - t <= QR_WINDOW_S]
                if now - last_hit_at.get(data, -999.0) >= QR_MIN_HIT_INTERVAL_S:
                    hist.append(now)
                    last_hit_at[data] = now
                    line = f"[QR] DATA={data!r} HITS={len(hist)}/{QR_REQUIRED_HITS}"
                    print(line, flush=True)
                    transcript.append(line)

                if len(hist) >= QR_REQUIRED_HITS:
                    result = "QR_VALID" if data == QR_EXPECTED_B2 else "QR_INVALID"
                    print(f"[QR] {result}: {data!r}", flush=True)
                    return {
                        "result": result,
                        "data": data,
                        "expected": QR_EXPECTED_B2,
                        "elapsed_s": time.monotonic() - t0,
                        "hits": len(hist),
                        "transcript_tail": transcript[-20:],
                    }

            time.sleep(0.01)

        print(f"[QR] QR_NOT_FOUND after {timeout_s:.1f}s", flush=True)
        return {
            "result": "QR_NOT_FOUND",
            "data": None,
            "expected": QR_EXPECTED_B2,
            "elapsed_s": time.monotonic() - t0,
            "hits": 0,
            "transcript_tail": transcript[-20:],
        }
    finally:
        cap.release()


# =============================================================================
# STM32 #2 ACTUATOR LINK - inherited unchanged from FIX8
# =============================================================================

# FIX10 actuator discovery policy:
# - never use the known STM32 #1 DRIVE device;
# - prefer STM32 by-id entries that are NOT STM32 #1;
# - fall back to remaining ttyACM devices only after excluding STM32 #1's resolved port;
# - send only VERSION while probing; no motion command is allowed until the exact
#   actuator firmware string is confirmed.
ACTUATOR_LEGACY_BY_ID = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_3692327B3435-if00"
)
DRIVE_STM32_BY_ID = (
    "/dev/serial/by-id/"
    "usb-STMicroelectronics_STM32_Virtual_ComPort_376B36513235-if00"
)
DRIVE_STM32_SERIAL_TOKEN = "376B36513235"
ACTUATOR_EXPECTED_VERSION = "AMR_ACTUATOR_F411_V2_12_PICK_LIFT_FIX"
ACTUATOR_BAUD = 115200
ACTUATOR_OPEN_TIMEOUT_S = 24.0
ACTUATOR_PICK_TIMEOUT_S = 50.0
ACTUATOR_DROP_TIMEOUT_S = 40.0
ACTUATOR_CLOSE_TIMEOUT_S = 30.0
ACTUATOR_STATUS_POLL_S = 0.25

MISSION_LOCK_PATH = "/tmp/amr_demo_fast_v3_fix8_actuator.lock"


class ActuatorClient:
    """Strict serial owner for STM32 #2. Never opens STM32 #1."""

    def __init__(self):
        self.ser = None
        self.last_status = {}
        self.port = None
        self.resolved_port = None
        self.detected_version = None

    @staticmethod
    def _candidate_ports() -> List[str]:
        blocked_real = set()
        drive_path = Path(DRIVE_STM32_BY_ID)
        if drive_path.exists():
            try:
                blocked_real.add(os.path.realpath(str(drive_path)))
            except Exception:
                pass

        candidates = []

        legacy = Path(ACTUATOR_LEGACY_BY_ID)
        if legacy.exists():
            candidates.append(str(legacy))

        by_id_dir = Path("/dev/serial/by-id")
        if by_id_dir.exists():
            for path in sorted(by_id_dir.iterdir(), key=lambda x: x.name):
                name = path.name
                if "STM32" not in name:
                    continue
                if DRIVE_STM32_SERIAL_TOKEN in name:
                    continue
                try:
                    real = os.path.realpath(str(path))
                except Exception:
                    continue
                if real in blocked_real:
                    continue
                candidates.append(str(path))

        # Fallback to ttyACM aliases only when STM32 #1's by-id is present, so
        # its resolved character device can be excluded with certainty. If that
        # identity is missing, fail closed instead of probing an unknown ttyACM.
        if blocked_real:
            for path in sorted(Path("/dev").glob("ttyACM*"), key=lambda x: x.name):
                real = os.path.realpath(str(path))
                if real in blocked_real:
                    continue
                candidates.append(str(path))

        # Deduplicate aliases resolving to the same character device.
        out = []
        seen_real = set()
        for candidate in candidates:
            try:
                real = os.path.realpath(candidate)
            except Exception:
                real = candidate
            if real in seen_real:
                continue
            seen_real.add(real)
            out.append(candidate)
        return out

    def connect(self):
        if serial is None:
            raise RuntimeError("pyserial is not installed; run: python3 -m pip install pyserial")
        if self.ser is not None and self.ser.is_open:
            return

        candidates = self._candidate_ports()
        if not candidates:
            raise RuntimeError("no safe STM32 #2 actuator candidate found")

        probe_errors = []
        for candidate in candidates:
            test_ser = None
            try:
                print(f"[ACT] probe candidate: {candidate}", flush=True)
                test_ser = serial.Serial(
                    candidate,
                    ACTUATOR_BAUD,
                    timeout=0.08,
                    write_timeout=0.5,
                )
                time.sleep(0.45)
                test_ser.reset_input_buffer()
                test_ser.reset_output_buffer()
                test_ser.write(b"VERSION\n")
                test_ser.flush()

                deadline = time.monotonic() + 1.5
                version = None
                while time.monotonic() < deadline:
                    raw = test_ser.readline()
                    if not raw:
                        continue
                    line = raw.decode(errors="replace").strip()
                    if line:
                        print(f"[ACT] {line}", flush=True)
                    if line.startswith("VERSION "):
                        version = line.replace("VERSION ", "", 1).strip()
                        break

                if version != ACTUATOR_EXPECTED_VERSION:
                    probe_errors.append(f"{candidate}: version={version!r}")
                    test_ser.close()
                    continue

                self.ser = test_ser
                self.port = candidate
                self.resolved_port = os.path.realpath(candidate)
                self.detected_version = version
                print(
                    f"[ACT] actuator selected: {self.port} -> {self.resolved_port}",
                    flush=True,
                )
                return

            except Exception as exc:
                probe_errors.append(f"{candidate}: {type(exc).__name__}: {exc}")
                try:
                    if test_ser is not None:
                        test_ser.close()
                except Exception:
                    pass

        raise RuntimeError(
            "STM32 #2 actuator not found with expected VERSION; "
            + " | ".join(probe_errors)
        )

    def close(self):
        try:
            if self.ser is not None and self.ser.is_open:
                self.ser.close()
        except Exception:
            pass
        self.ser = None

    def _send(self, cmd: str):
        if self.ser is None or not self.ser.is_open:
            raise RuntimeError("actuator serial is not open")
        print(f"[ACT] >>> {cmd}", flush=True)
        self.ser.write((cmd.strip() + "\n").encode("ascii"))
        self.ser.flush()

    @staticmethod
    def _parse_status(line: str) -> Dict[str, int]:
        if not line.startswith("STATUS "):
            return {}
        out = {}
        for key, val in re.findall(r"([A-Z_]+)=(-?\d+)", line):
            try:
                out[key] = int(val)
            except ValueError:
                pass
        return out

    def _read_line(self) -> Optional[str]:
        raw = self.ser.readline()
        if not raw:
            return None
        line = raw.decode(errors="replace").strip()
        if line:
            print(f"[ACT] {line}", flush=True)
        return line or None

    def _drain(self, duration_s: float = 0.12):
        end = time.monotonic() + duration_s
        while time.monotonic() < end:
            line = self._read_line()
            if line is None:
                continue

    def _query_line(self, cmd: str, prefix: str, timeout_s: float = 1.5) -> str:
        self._drain(0.08)
        self._send(cmd)
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            line = self._read_line()
            if line is None:
                continue
            if line.startswith("ERR "):
                raise RuntimeError(f"{cmd} rejected: {line}")
            if line.startswith(prefix):
                return line
        raise RuntimeError(f"timeout waiting {prefix!r} after {cmd}")

    def status(self) -> Dict[str, int]:
        line = self._query_line("STATUS", "STATUS ", timeout_s=1.5)
        st = self._parse_status(line)
        self.last_status = st
        return st

    def verify_ready(self) -> Dict:
        self.connect()
        version = self.detected_version
        if version != ACTUATOR_EXPECTED_VERSION:
            raise RuntimeError(
                f"wrong actuator firmware: {version!r}; expected {ACTUATOR_EXPECTED_VERSION!r}"
            )
        st = self.status()
        if st.get("FAULT", -1) != 0:
            raise RuntimeError(f"actuator FAULT at preflight: {st}")
        if st.get("BUSY", -1) != 0:
            raise RuntimeError(f"actuator BUSY at preflight: {st}")
        return {
            "result": "PASS",
            "port": self.port,
            "resolved_port": self.resolved_port,
            "version": version,
            "status": st,
        }

    def _wait_motion(
        self,
        cmd: str,
        done_token: str,
        done_state: int,
        timeout_s: float,
    ) -> Dict:
        self._drain(0.08)
        self._send(cmd)
        t0 = time.monotonic()
        next_poll = t0 + ACTUATOR_STATUS_POLL_S
        transcript = []
        final_status = None

        while time.monotonic() - t0 < timeout_s:
            line = self._read_line()
            if line is not None:
                transcript.append(line)
                if line.startswith("ERR "):
                    raise RuntimeError(f"{cmd} rejected: {line}")
                m_fault = re.fullmatch(r"FAULT\s+(\d+)", line)
                if m_fault and int(m_fault.group(1)) != 0:
                    raise RuntimeError(f"{cmd} actuator fault event: {line}")
                if line == done_token:
                    final_status = self.status()
                    if final_status.get("FAULT", 0) != 0:
                        raise RuntimeError(f"{cmd} ended with fault: {final_status}")
                    return {
                        "result": "PASS",
                        "command": cmd,
                        "done": done_token,
                        "elapsed_s": time.monotonic() - t0,
                        "status": final_status,
                        "transcript_tail": transcript[-20:],
                    }
                st = self._parse_status(line)
                if st:
                    self.last_status = st
                    final_status = st
                    if st.get("FAULT", 0) != 0:
                        raise RuntimeError(f"{cmd} actuator fault: {st}")
                    if st.get("STATE") == done_state and st.get("BUSY", 0) == 0:
                        return {
                            "result": "PASS",
                            "command": cmd,
                            "done": f"STATE={done_state}",
                            "elapsed_s": time.monotonic() - t0,
                            "status": st,
                            "transcript_tail": transcript[-20:],
                        }

            now = time.monotonic()
            if now >= next_poll:
                self._send("STATUS")
                next_poll = now + ACTUATOR_STATUS_POLL_S

        self.safe_stop()
        raise RuntimeError(f"{cmd} timeout after {timeout_s:.1f}s; last_status={final_status}")

    def open_full(self) -> Dict:
        return self._wait_motion("OPEN", "OPEN_DONE", 11, ACTUATOR_OPEN_TIMEOUT_S)

    def pick(self) -> Dict:
        return self._wait_motion("PICK", "PICK_DONE", 22, ACTUATOR_PICK_TIMEOUT_S)

    def drop(self) -> Dict:
        return self._wait_motion("DROP", "DROP_DONE", 32, ACTUATOR_DROP_TIMEOUT_S)

    def close_empty_after_drop(self) -> Dict:
        # DROP_DONE is a full-open state in the current STM32 firmware.
        st = self.status()
        if st.get("FAULT", 0) != 0:
            raise RuntimeError(f"cannot close after drop; actuator fault: {st}")
        if st.get("OPEN_FULL") != 1:
            raise RuntimeError(f"cannot close after drop; OPEN_FULL not confirmed: {st}")

        ack = self._query_line("ARM_CLOSE", "ACK ARM_CLOSE", timeout_s=1.5)
        rec = self._wait_motion("CLOSE", "CLOSE_DONE", 13, ACTUATOR_CLOSE_TIMEOUT_S)
        rec["arm_ack"] = ack
        return rec

    def safe_stop(self):
        try:
            if self.ser is None or not self.ser.is_open:
                return
            self._send("STOP")
            end = time.monotonic() + 0.6
            while time.monotonic() < end:
                self._read_line()
        except Exception as exc:
            print(f"[ACT] STOP warning: {exc}", flush=True)


# =============================================================================
# ROS NODE
# =============================================================================

class DemoFastNode(Node):
    def __init__(self):
        super().__init__("amr_demo_fast_v3_fix10_b2_qr")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.nav_client = ActionClient(self, NavigateToPose, NAV_ACTION)
        self.drive_client = ActionClient(self, DriveOnHeading, DRIVE_ACTION)
        self.backup_client = ActionClient(self, BackUp, BACKUP_ACTION)

        self.turn_pub = self.create_publisher(Vector3Stamped, TURN_TOPIC, 10)

        # Dynamic footprint publishers. Nav2 costmaps accept geometry_msgs/Polygon
        # on their footprint input topics. EMPTY is published at startup; after
        # PICK the profile switches to LOADED, and only returns to EMPTY after
        # the DROP + 0.40 m retreat to TH has completed.
        self.local_footprint_pub = self.create_publisher(
            Polygon, LOCAL_FOOTPRINT_TOPIC, 10
        )
        self.global_footprint_pub = self.create_publisher(
            Polygon, GLOBAL_FOOTPRINT_TOPIC, 10
        )
        load_state_qos = QoSProfile(depth=1)
        load_state_qos.reliability = ReliabilityPolicy.RELIABLE
        load_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.load_state_pub = self.create_publisher(
            String, LOAD_STATE_TOPIC, load_state_qos
        )
        self.load_profile = "EMPTY"

        self.clear_local_client = self.create_client(
            ClearEntireCostmap,
            "/local_costmap/clear_entirely_local_costmap",
        )
        self.clear_global_client = self.create_client(
            ClearEntireCostmap,
            "/global_costmap/clear_entirely_global_costmap",
        )

        self.latest_scan: Optional[LaserScan] = None
        self.last_scan_mono = 0.0
        self.latest_odom: Optional[Odometry] = None
        self.last_odom_mono = 0.0

        self.latest_cmd_safe: Optional[Twist] = None
        self.last_cmd_safe_mono = 0.0

        self.latest_cmd_nav: Optional[Twist] = None
        self.last_cmd_nav_mono = 0.0

        # If Ctrl+C happens during turn_relative, the bridge intentionally
        # ignores stale/zero cmd_vel while the explicit turn is active.
        # Keep a local window so SAFE STOP can explicitly cancel that turn.
        self.turn_active_until_mono = 0.0

        self.create_subscription(LaserScan, SCAN_TOPIC, self._scan_cb, 20)
        self.create_subscription(Odometry, ODOM_TOPIC, self._odom_cb, 20)
        self.create_subscription(Twist, CMD_SAFE, self._cmd_safe_cb, 20)
        # FIX7: the route-blocked watchdog uses fresh Nav2 motion intent.
        # FIX6 defined _cmd_nav_cb() but never subscribed to /cmd_vel_nav, so
        # motion_intent_seen could never arm reliably. This is observation-only.
        self.create_subscription(Twist, CMD_NAV, self._cmd_nav_cb, 20)

        # Emergency-only publisher. It is used only to cancel an active
        # turn_relative at the bridge, then immediately sends zeros.
        self.safe_stop_pub = self.create_publisher(Twist, CMD_SAFE, 10)

        self.last_drive_feedback = None
        self.last_backup_feedback = None

    def _scan_cb(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.last_scan_mono = time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.last_odom_mono = time.monotonic()

    def _cmd_safe_cb(self, msg: Twist) -> None:
        self.latest_cmd_safe = msg
        self.last_cmd_safe_mono = time.monotonic()

    def _cmd_nav_cb(self, msg: Twist) -> None:
        self.latest_cmd_nav = msg
        self.last_cmd_nav_mono = time.monotonic()

    @staticmethod
    def _profile_physical_bounds(profile: str) -> Tuple[float, float, float, float]:
        p = profile.upper()
        if p == "LOADED":
            return (
                LOADED_FRONT_M, LOADED_REAR_M,
                LOADED_LEFT_M, LOADED_RIGHT_M,
            )
        return (
            EMPTY_FRONT_M, EMPTY_REAR_M,
            EMPTY_LEFT_M, EMPTY_RIGHT_M,
        )

    def _profile_padded_bounds(self) -> Tuple[float, float, float, float]:
        front, rear, left, right = self._profile_physical_bounds(self.load_profile)
        return (
            front + FOOTPRINT_PADDING_M,
            rear + FOOTPRINT_PADDING_M,
            left + FOOTPRINT_PADDING_M,
            right + FOOTPRINT_PADDING_M,
        )

    @staticmethod
    def _polygon_for_profile(profile: str) -> Polygon:
        front, rear, left, right = DemoFastNode._profile_physical_bounds(profile)
        poly = Polygon()
        for x, y in (
            (front, left),
            (front, -right),
            (-rear, -right),
            (-rear, left),
        ):
            p = Point32()
            p.x = float(x)
            p.y = float(y)
            p.z = 0.0
            poly.points.append(p)
        return poly

    def set_load_profile(self, profile: str, settle_s: float = 0.35) -> Dict:
        profile = profile.upper()
        if profile not in ("EMPTY", "LOADED"):
            raise ValueError(f"invalid load profile: {profile}")

        poly = self._polygon_for_profile(profile)
        state = String()
        state.data = profile

        # Repeat briefly so both costmaps + dynamic turn gate receive the state.
        for _ in range(3):
            self.local_footprint_pub.publish(poly)
            self.global_footprint_pub.publish(poly)
            self.load_state_pub.publish(state)
            rclpy.spin_once(self, timeout_sec=0.03)
            time.sleep(0.04)

        self.load_profile = profile
        self.spin_for(settle_s)

        front, rear, left, right = self._profile_physical_bounds(profile)
        print(
            f"FOOTPRINT PROFILE -> {profile}: "
            f"front=+{front:.3f} rear=-{rear:.3f} "
            f"left=+{left:.3f} right=-{right:.3f} m "
            f"(costmap padding={FOOTPRINT_PADDING_M:.2f} m)"
        )
        return {
            "profile": profile,
            "front_m": front,
            "rear_m": rear,
            "left_m": left,
            "right_m": right,
            "padding_m": FOOTPRINT_PADDING_M,
        }

    def clear_costmaps_and_rescan(self, settle_s: float = 0.80) -> Dict:
        rec = {}
        for name, client in (
            ("local", self.clear_local_client),
            ("global", self.clear_global_client),
        ):
            if not client.wait_for_service(timeout_sec=1.5):
                rec[name] = "SERVICE_UNAVAILABLE"
                continue
            fut = client.call_async(ClearEntireCostmap.Request())
            result = self.wait_future(fut, 2.0)
            rec[name] = "OK" if result is not None else "TIMEOUT"

        # Do not plan immediately on an empty costmap. Give /scan + costmap time
        # to repopulate persistent obstacles before resending the station goal.
        self.spin_for(settle_s)
        age = (
            time.monotonic() - self.last_scan_mono
            if self.last_scan_mono > 0.0 else float("inf")
        )
        rec["scan_age_s_after_rescan"] = age
        rec["scan_fresh_after_rescan"] = age <= 0.40
        return rec

    def spin_for(self, duration_s: float) -> None:
        end = time.monotonic() + duration_s
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_future(self, future, timeout_s: float):
        end = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            return None
        return future.result()

    def map_pose_once(self) -> Optional[Tuple[float, float, float]]:
        try:
            tr = self.tf_buffer.lookup_transform(
                MAP_FRAME,
                BASE_FRAME,
                Time(),
            )
        except TransformException:
            return None
        return (
            float(tr.transform.translation.x),
            float(tr.transform.translation.y),
            yaw_from_quat(tr.transform.rotation),
        )

    def stable_map_pose(
        self,
        samples: int = 12,
        timeout_s: float = 4.0,
    ) -> Optional[Tuple[float, float, float]]:
        values = []
        deadline = time.monotonic() + timeout_s

        while rclpy.ok() and len(values) < samples and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            p = self.map_pose_once()
            if p is not None:
                values.append(p)
            time.sleep(0.03)

        if len(values) < max(5, samples // 2):
            return None

        xs = [p[0] for p in values]
        ys = [p[1] for p in values]
        yaws = [p[2] for p in values]

        return (
            statistics.median(xs),
            statistics.median(ys),
            circular_mean(yaws),
        )

    def wait_robot_stopped(
        self,
        timeout_s: float = 4.0,
        stable_s: float = 0.45,
        v_tol: float = 0.01,
        w_tol: float = 0.012,
    ) -> bool:
        deadline = time.monotonic() + timeout_s
        stable_since = None

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            msg = self.latest_odom
            if msg is None:
                continue

            v = abs(float(msg.twist.twist.linear.x))
            w = abs(float(msg.twist.twist.angular.z))

            if v <= v_tol and w <= w_tol:
                if stable_since is None:
                    stable_since = time.monotonic()
                elif time.monotonic() - stable_since >= stable_s:
                    return True
            else:
                stable_since = None

        return False

    # -------------------------------------------------------------------------
    # SAFE STOP: cancel every motion authority BEFORE bag shutdown / node exit
    # -------------------------------------------------------------------------

    def _cancel_all_for_action(
        self,
        action_name: str,
        timeout_s: float = 1.0,
    ) -> Dict:
        service_name = f"{action_name}/_action/cancel_goal"
        client = self.create_client(CancelGoal, service_name)

        rec = {
            "action": action_name,
            "service": service_name,
            "available": False,
            "return_code": None,
            "goals_canceling": None,
        }

        try:
            if not client.wait_for_service(timeout_sec=0.5):
                return rec

            rec["available"] = True

            # Default CancelGoal request = zero UUID + zero stamp,
            # which means cancel ALL goals on this action server.
            req = CancelGoal.Request()
            future = client.call_async(req)

            deadline = time.monotonic() + timeout_s
            while rclpy.ok() and not future.done() and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.03)

            if not future.done():
                rec["timeout"] = True
                return rec

            response = future.result()
            rec["return_code"] = int(response.return_code)
            rec["goals_canceling"] = len(response.goals_canceling)
            return rec

        except Exception as exc:
            rec["error"] = f"{type(exc).__name__}: {exc}"
            return rec

        finally:
            try:
                self.destroy_client(client)
            except Exception:
                pass

    def _cancel_active_turn_relative(self) -> Dict:
        """
        Bridge source policy:
          - stale/zero cmd_vel does NOT cancel an active turn_relative
          - a subsequent NON-ZERO cmd_vel cancels the turn

        Therefore, only when our own turn window is still active, send a
        microscopic non-zero safe command to force the bridge out of turn mode,
        then immediately send repeated zeros. This is emergency shutdown logic,
        not normal driving.
        """
        rec = {
            "needed": time.monotonic() < self.turn_active_until_mono,
            "pulse_sent": False,
        }

        if not rec["needed"]:
            return rec

        print("SAFE STOP: active turn_relative window detected")
        print("SAFE STOP: canceling explicit STM32 turn, then forcing zero")

        pulse = Twist()
        # 0.00002 m/s -> effectively zero mechanically, but non-zero at the
        # bridge command-latch threshold so it explicitly cancels turn mode.
        pulse.linear.x = 0.00002
        pulse.angular.z = 0.0

        zero = Twist()

        # Allow DDS discovery for this emergency publisher.
        end = time.monotonic() + 0.20
        while rclpy.ok() and time.monotonic() < end:
            self.safe_stop_pub.publish(pulse)
            rclpy.spin_once(self, timeout_sec=0.02)

        rec["pulse_sent"] = True

        end = time.monotonic() + 0.35
        while rclpy.ok() and time.monotonic() < end:
            self.safe_stop_pub.publish(zero)
            rclpy.spin_once(self, timeout_sec=0.02)

        self.turn_active_until_mono = 0.0
        return rec

    def safe_stop_all(
        self,
        reason: str,
        wait_stop_s: float = 4.0,
    ) -> Dict:
        """
        Idempotent software stop sequence.

        Order matters:
          1) Cancel NavigateToPose parent action.
          2) Cancel child behavior actions directly (DriveOnHeading, BackUp, Spin).
          3) Repeat once to close parent/child cancellation races.
          4) If an explicit turn_relative may still be active, cancel it at bridge.
          5) Verify odometry becomes stationary.

        This runs BEFORE rosbag shutdown.
        """
        print()
        print("!" * 72)
        print(f"SAFE STOP START: {reason}")
        print("Canceling ALL ROS motion actions before process shutdown...")
        print("!" * 72)

        actions = (
            NAV_ACTION,
            DRIVE_ACTION,
            BACKUP_ACTION,
            "/spin",
        )

        all_results = []

        # First cancel parent, then every possible moving child.
        for action_name in actions:
            rec = self._cancel_all_for_action(action_name)
            all_results.append(rec)
            print(
                f"  cancel {action_name}: "
                f"available={rec.get('available')} "
                f"code={rec.get('return_code')} "
                f"goals={rec.get('goals_canceling')}"
            )

        # Give BT/behavior_server a moment to process HALT, then sweep again.
        settle_end = time.monotonic() + 0.25
        while rclpy.ok() and time.monotonic() < settle_end:
            rclpy.spin_once(self, timeout_sec=0.03)

        for action_name in actions:
            rec = self._cancel_all_for_action(action_name)
            all_results.append(rec)

        turn_rec = self._cancel_active_turn_relative()

        stopped = self.wait_robot_stopped(
            timeout_s=wait_stop_s,
            stable_s=0.55,
            v_tol=0.008,
            w_tol=0.010,
        )

        cmd_rec = {
            "recent": False,
            "linear_x": None,
            "angular_z": None,
        }
        if self.latest_cmd_safe is not None:
            age = time.monotonic() - self.last_cmd_safe_mono
            cmd_rec["recent"] = age <= 0.50
            cmd_rec["linear_x"] = float(self.latest_cmd_safe.linear.x)
            cmd_rec["angular_z"] = float(self.latest_cmd_safe.angular.z)

        if stopped:
            print("SAFE STOP PASS: odom stationary")
        else:
            print("SAFE STOP FAIL: odom did not confirm stationary")
            print("USE HARDWARE E-STOP / MOTOR ENABLE CUT. DO NOT CONTINUE TEST.")

        print("!" * 72)

        return {
            "reason": reason,
            "action_cancels": all_results,
            "turn_cancel": turn_rec,
            "odom_stationary": bool(stopped),
            "last_cmd_safe": cmd_rec,
        }


    # -------------------------------------------------------------------------
    # Graph / lifecycle preflight
    # -------------------------------------------------------------------------

    def canonical_node_names(self) -> set:
        names = set()
        try:
            for name, namespace in self.get_node_names_and_namespaces():
                ns = (namespace or "/").rstrip("/")
                if ns in ("", "/"):
                    fq = f"/{name.lstrip('/')}"
                else:
                    fq = f"{ns}/{name.lstrip('/')}"
                names.add(fq)
        except Exception:
            pass
        return names

    def wait_required_nodes(self, timeout_s: float = 5.0) -> Tuple[bool, List[str]]:
        deadline = time.monotonic() + timeout_s
        names = set()

        while rclpy.ok() and time.monotonic() < deadline:
            self.spin_for(0.20)
            names = self.canonical_node_names()
            if REQUIRED_NODES.issubset(names):
                return True, []
        return False, sorted(REQUIRED_NODES - names)

    def lifecycle_active(self, node_name: str, timeout_s: float = 2.5) -> bool:
        service = f"{node_name}/get_state"
        client = self.create_client(GetState, service)

        if not client.wait_for_service(timeout_sec=timeout_s):
            self.destroy_client(client)
            return False

        future = client.call_async(GetState.Request())
        result = self.wait_future(future, timeout_s)
        self.destroy_client(client)

        return bool(result and result.current_state.id == 3)


    @staticmethod
    def _endpoint_fq_names(infos) -> set:
        names = set()
        for info in infos:
            name = getattr(info, "node_name", "") or ""
            ns = getattr(info, "node_namespace", "/") or "/"
            if not name:
                continue
            ns = ns.rstrip("/")
            if ns in ("", "/"):
                names.add(f"/{name.lstrip('/')}")
            else:
                names.add(f"{ns}/{name.lstrip('/')}")
        return names

    def command_chain_preflight(self) -> Tuple[bool, Dict]:
        """Verify the expected safety-command ownership without commanding motion."""
        checks = {}

        def pubs(topic: str) -> set:
            return self._endpoint_fq_names(self.get_publishers_info_by_topic(topic))

        def subs(topic: str) -> set:
            return self._endpoint_fq_names(self.get_subscriptions_info_by_topic(topic))

        expected = [
            ("/cmd_vel_nav", "pub", "/behavior_server"),
            ("/cmd_vel_nav", "sub", "/velocity_smoother"),
            ("/cmd_vel", "pub", "/velocity_smoother"),
            ("/cmd_vel", "sub", "/amr_turn_safety_gate_v3_dynamic"),
            ("/cmd_vel_guarded", "pub", "/amr_turn_safety_gate_v3_dynamic"),
            ("/cmd_vel_guarded", "sub", "/collision_monitor"),
            ("/cmd_vel_safe", "pub", "/collision_monitor"),
            ("/cmd_vel_safe", "sub", "/amr_stm32_bridge"),
        ]

        ok = True
        for topic, kind, node_name in expected:
            got = pubs(topic) if kind == "pub" else subs(topic)
            key = f"{topic}:{kind}:{node_name}"
            passed = node_name in got
            checks[key] = {"pass": passed, "seen": sorted(got)}
            ok = ok and passed

        return ok, checks

    def sensor_freshness_preflight(self) -> Tuple[bool, Dict]:
        self.spin_for(1.0)
        now = time.monotonic()
        odom_age = None if self.last_odom_mono <= 0.0 else now - self.last_odom_mono
        scan_age = None if self.last_scan_mono <= 0.0 else now - self.last_scan_mono
        ok = (
            odom_age is not None and odom_age <= 0.30
            and scan_age is not None and scan_age <= 0.40
        )
        return ok, {
            "odom_age_s": odom_age,
            "scan_age_s": scan_age,
            "odom_limit_s": 0.30,
            "scan_limit_s": 0.40,
        }

    # -------------------------------------------------------------------------
    # NavigateToPose
    # -------------------------------------------------------------------------

    def _make_pose_stamped(self, cfg: Dict[str, float]):
        from geometry_msgs.msg import PoseStamped

        msg = PoseStamped()
        msg.header.frame_id = MAP_FRAME
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.pose.position.x = float(cfg["x"])
        msg.pose.position.y = float(cfg["y"])
        msg.pose.position.z = 0.0

        yaw = math.radians(float(cfg["yaw_deg"]))
        msg.pose.orientation.z = math.sin(0.5 * yaw)
        msg.pose.orientation.w = math.cos(0.5 * yaw)
        return msg

    def navigate_to_station(
        self,
        name: str,
        cfg: Dict[str, float],
        bt_path: str,
        timeout_s: float = 240.0,
    ) -> Tuple[bool, Dict]:
        """
        Navigate with Theta* + MPPI + recovery, but do NOT let Nav2 spend time
        hunting the final station yaw.

        Handoff rule:
          XY <= NAV_POSITION_HANDOFF_M
          AND |yaw error| <= NAV_YAW_HANDOFF_DEG
          continuously for NAV_HANDOFF_STABLE_S

        Once the envelope is reached:
          cancel NavigateToPose -> wait robot stopped -> return SUCCESS_HANDOFF.
        The mission then performs exactly one STM32 turn_relative correction.
        """
        print()
        print("=" * 68)
        print(f"NAVIGATE -> {name}")
        print(
            f"TARGET x={float(cfg['x']):+.6f} "
            f"y={float(cfg['y']):+.6f} "
            f"yaw={float(cfg['yaw_deg']):+.3f} deg"
        )
        handoff_xy_m = STATION_HANDOFF_XY_M.get(
            name, NAV_POSITION_HANDOFF_M
        )
        terminal_guard_m = handoff_xy_m + TERMINAL_GUARD_EXTRA_M

        print(
            f"HANDOFF: XY<={handoff_xy_m*100.0:.1f} cm "
            f"AND |YAW|<={NAV_YAW_HANDOFF_DEG:.1f} deg "
            f"for {NAV_HANDOFF_STABLE_S:.2f}s"
        )
        print(
            f"ANTI-HUNT GUARD: once XY<={terminal_guard_m*100.0:.1f} cm, "
            f"Nav2 has max {TERMINAL_GUARD_MAX_S:.1f}s to handoff; "
            "otherwise CANCEL + STOP."
        )
        print(f"BT: {bt_path}")
        print("=" * 68)

        if not self.nav_client.wait_for_server(timeout_sec=5.0):
            return False, {"reason": "navigate_to_pose server unavailable"}

        goal = NavigateToPose.Goal()
        goal.pose = self._make_pose_stamped(cfg)
        goal.behavior_tree = bt_path

        send_future = self.nav_client.send_goal_async(goal)
        goal_handle = self.wait_future(send_future, 8.0)

        if goal_handle is None or not goal_handle.accepted:
            return False, {"reason": "goal rejected or send timeout"}

        print("GOAL ACCEPTED")

        result_future = goal_handle.get_result_async()
        goal_accept_mono = time.monotonic()
        deadline = goal_accept_mono + timeout_s
        handoff_since = None
        terminal_since = None
        blocked_since = None
        blocked_anchor_pose = self.map_pose_once()
        motion_intent_seen = False
        last_report = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            # If Nav2 finished by itself first, use the normal result.
            if result_future.done():
                wrapped = result_future.result()
                status = int(wrapped.status)

                self.wait_robot_stopped(timeout_s=3.0)
                pose = self.stable_map_pose()

                rec = {
                    "status": status,
                    "handoff": False,
                    "completion": "NAV2_RESULT",
                }

                if pose is not None:
                    xy, yaw_err = pose_error(pose, cfg)
                    rec.update(
                        {
                            "final_x": pose[0],
                            "final_y": pose[1],
                            "final_yaw_deg": math.degrees(pose[2]),
                            "xy_error_cm": xy * 100.0,
                            "yaw_error_deg": math.degrees(yaw_err),
                        }
                    )
                    print(
                        f"FINAL MAP: x={pose[0]:+.3f} y={pose[1]:+.3f} "
                        f"yaw={math.degrees(pose[2]):+.3f} deg"
                    )
                    print(
                        f"ERROR: XY={xy*100.0:.2f} cm | "
                        f"YAW={math.degrees(yaw_err):+.3f} deg"
                    )

                print(f"NAV2 RESULT STATUS = {status}")

                if status == GoalStatus.STATUS_SUCCEEDED:
                    return True, rec

                # FIX7: Nav2 ABORTED (status=6) is no longer an immediate
                # mission-kill. Stop all residual motion authority, return a
                # recoverable reason, then navigate_with_escape_recovery() may
                # perform ONE bounded BackUp/rescan/replan of the SAME goal.
                if status == GoalStatus.STATUS_ABORTED:
                    stop_rec = self.safe_stop_all(
                        f"NAV2_ABORTED_{name}",
                        wait_stop_s=3.0,
                    )
                    rec["reason"] = "nav2 aborted status 6"
                    rec["safe_stop"] = stop_rec
                    return False, rec

                rec["reason"] = f"nav2 result status {status}"
                return False, rec

            pose = self.map_pose_once()
            if pose is None:
                handoff_since = None
                continue

            xy, yaw_err = pose_error(pose, cfg)
            yaw_err_deg = math.degrees(yaw_err)

            eligible = (
                xy <= handoff_xy_m
                and abs(yaw_err_deg) <= NAV_YAW_HANDOFF_DEG
            )

            now = time.monotonic()

            # Near-goal anti-hunt / route-blocked detection.
            #
            # FIX2 rationale from the 2026-09-14 18:47:31 rosbag:
            # - Collision Monitor legitimately forced /cmd_vel_safe to zero for
            #   ~0.65 s while /cmd_vel_guarded remained non-zero.
            # - The robot then resumed motion, but Nav2 later stopped issuing
            #   motion and the robot stayed nearly stationary long enough for the
            #   old 2.5 s mission watchdog to preempt Nav2 recovery/replanning.
            # - On the retry there was no new /cmd_vel_nav motion output before
            #   V3 aborted, so the watchdog must not arm before Nav2 has actually
            #   produced fresh motion intent.
            # - Freshness/progress checks are also added so delayed callbacks or
            #   instantaneous low-speed samples cannot create another false block.
            odom_v = 0.0
            odom_w = 0.0
            odom_age = float("inf")
            if self.latest_odom is not None and self.last_odom_mono > 0.0:
                odom_v = abs(float(self.latest_odom.twist.twist.linear.x))
                odom_w = abs(float(self.latest_odom.twist.twist.angular.z))
                odom_age = now - self.last_odom_mono
            odom_fresh = odom_age <= ROUTE_BLOCKED_ODOM_FRESH_S

            cmd_nav_v = 0.0
            cmd_nav_w = 0.0
            cmd_nav_age = float("inf")
            if self.latest_cmd_nav is not None and self.last_cmd_nav_mono > 0.0:
                cmd_nav_v = float(self.latest_cmd_nav.linear.x)
                cmd_nav_w = float(self.latest_cmd_nav.angular.z)
                cmd_nav_age = now - self.last_cmd_nav_mono
            cmd_nav_fresh = cmd_nav_age <= ROUTE_BLOCKED_CMD_FRESH_S
            if cmd_nav_fresh and (
                abs(cmd_nav_v) > 0.01 or abs(cmd_nav_w) > 0.02
            ):
                motion_intent_seen = True

            # If odom is stale, treat motion as UNKNOWN (not stationary) so a
            # callback backlog cannot create a false anti-hunt/blocked trigger.
            near_goal_motion = (
                (not odom_fresh)
                or (odom_v > 0.012)
                or (odom_w > 0.018)
            )

            # Real pose progress resets the blocked timer even if instantaneous
            # odom happens to be small at the sampling instant.
            pose_progressed = False
            if blocked_anchor_pose is None:
                blocked_anchor_pose = pose
            else:
                pdx = pose[0] - blocked_anchor_pose[0]
                pdy = pose[1] - blocked_anchor_pose[1]
                pdyaw = abs(wrap_pi(pose[2] - blocked_anchor_pose[2]))
                if (
                    math.hypot(pdx, pdy) >= ROUTE_PROGRESS_RESET_M
                    or math.degrees(pdyaw) >= ROUTE_PROGRESS_RESET_YAW_DEG
                ):
                    pose_progressed = True
                    blocked_anchor_pose = pose
                    blocked_since = None

            low_motion = (
                odom_v <= ROUTE_BLOCKED_V_MAX_MPS
                and odom_w <= ROUTE_BLOCKED_W_MAX_RAD_S
            )
            watchdog_eligible = (
                xy > terminal_guard_m
                and (now - goal_accept_mono) >= ROUTE_BLOCKED_START_GRACE_S
                and motion_intent_seen
                and odom_fresh
                and low_motion
                and not pose_progressed
            )

            if watchdog_eligible:
                if blocked_since is None:
                    blocked_since = now
                elif now - blocked_since >= ROUTE_BLOCKED_MAX_S:
                    print()
                    print(
                        f"ROUTE BLOCKED -> {name}: confirmed no-progress "
                        f"{now-blocked_since:.1f}s at XY={xy*100.0:.1f} cm "
                        f"(odom_age={odom_age:.3f}s cmd_nav_age={cmd_nav_age:.3f}s)"
                    )
                    try:
                        self.wait_future(goal_handle.cancel_goal_async(), 2.0)
                    except Exception:
                        pass
                    stop_rec = self.safe_stop_all(
                        f"ROUTE_BLOCKED_{name}",
                        wait_stop_s=4.0,
                    )
                    return False, {
                        "reason": "route blocked/no progress",
                        "xy_error_cm": xy * 100.0,
                        "yaw_error_deg": yaw_err_deg,
                        "blocked_stationary_s": now - blocked_since,
                        "odom_age_s": odom_age,
                        "cmd_nav_age_s": cmd_nav_age,
                        "safe_stop": stop_rec,
                    }
            else:
                blocked_since = None

            if xy <= terminal_guard_m and not eligible:
                if near_goal_motion:
                    terminal_since = None
                elif terminal_since is None:
                    terminal_since = now
                    print(
                        f"ANTI-HUNT ARMED -> {name}: "
                        f"XY={xy*100.0:.1f} cm | stationary near goal"
                    )
                elif now - terminal_since >= TERMINAL_GUARD_MAX_S:
                    print()
                    print(
                        f"ANTI-HUNT TRIGGER -> {name}: "
                        f"near goal for {now-terminal_since:.1f}s "
                        f"without valid handoff."
                    )
                    print("CANCEL Nav2 + all child motion actions. NO MORE HUNTING.")

                    try:
                        self.wait_future(goal_handle.cancel_goal_async(), 2.0)
                    except Exception:
                        pass

                    stop_rec = self.safe_stop_all(
                        f"TERMINAL_HUNT_GUARD_{name}",
                        wait_stop_s=4.0,
                    )

                    return False, {
                        "reason": "terminal anti-hunt guard triggered",
                        "xy_error_cm": xy * 100.0,
                        "yaw_error_deg": yaw_err_deg,
                        "safe_stop": stop_rec,
                    }
            else:
                terminal_since = None

            if now - last_report >= 1.0:
                print(
                    f"APPROACH {name}: XY={xy*100.0:.1f} cm | "
                    f"YAW={yaw_err_deg:+.2f} deg | "
                    f"handoff={'YES' if eligible else 'NO'}"
                )
                last_report = now

            if eligible:
                if handoff_since is None:
                    handoff_since = now

                if now - handoff_since >= NAV_HANDOFF_STABLE_S:
                    print()
                    print(
                        f"NAV HANDOFF -> {name}: "
                        f"XY={xy*100.0:.2f} cm | "
                        f"YAW={yaw_err_deg:+.3f} deg"
                    )
                    print(
                        "CANCEL NavigateToPose BEFORE terminal yaw hunting; "
                        "final yaw will use ONE-SHOT turn_relative."
                    )

                    cancel_future = goal_handle.cancel_goal_async()
                    cancel_result = self.wait_future(cancel_future, 3.0)

                    if cancel_result is None:
                        return False, {
                            "reason": "handoff cancel timeout",
                            "xy_error_cm": xy * 100.0,
                            "yaw_error_deg": yaw_err_deg,
                        }

                    goals_canceling = getattr(
                        cancel_result,
                        "goals_canceling",
                        [],
                    )

                    # Race: Nav2 may have succeeded while cancel was requested.
                    if not goals_canceling and result_future.done():
                        wrapped = result_future.result()
                        status = int(wrapped.status)
                        self.wait_robot_stopped(timeout_s=3.0)
                        return (
                            status == GoalStatus.STATUS_SUCCEEDED,
                            {
                                "status": status,
                                "handoff": False,
                                "completion": "NAV2_RESULT_DURING_HANDOFF",
                            },
                        )

                    if not goals_canceling:
                        return False, {
                            "reason": "handoff cancel rejected",
                            "xy_error_cm": xy * 100.0,
                            "yaw_error_deg": yaw_err_deg,
                        }

                    # Wait briefly for the canceled result so controller output
                    # is no longer active before turn_relative is allowed.
                    self.wait_future(result_future, 4.0)

                    if not self.wait_robot_stopped(timeout_s=4.0):
                        return False, {
                            "reason": "robot did not stop after Nav2 handoff",
                            "xy_error_cm": xy * 100.0,
                            "yaw_error_deg": yaw_err_deg,
                        }

                    final_pose = self.stable_map_pose()
                    if final_pose is None:
                        return False, {
                            "reason": "map pose unavailable after handoff"
                        }

                    final_xy, final_yaw_err = pose_error(final_pose, cfg)
                    final_yaw_err_deg = math.degrees(final_yaw_err)

                    print(
                        f"HANDOFF STOPPED: XY={final_xy*100.0:.2f} cm | "
                        f"YAW={final_yaw_err_deg:+.3f} deg"
                    )

                    # The one-shot aligner refuses >8 deg, so enforce that here.
                    if (
                        final_xy > handoff_xy_m + 0.02
                        or abs(final_yaw_err_deg) > YAW_MAX_CORRECTION_DEG
                    ):
                        return False, {
                            "reason": "handoff drifted outside safe envelope",
                            "xy_error_cm": final_xy * 100.0,
                            "yaw_error_deg": final_yaw_err_deg,
                        }

                    return True, {
                        "status": GoalStatus.STATUS_CANCELED,
                        "handoff": True,
                        "completion": "POSITION_COARSE_YAW_HANDOFF",
                        "final_x": final_pose[0],
                        "final_y": final_pose[1],
                        "final_yaw_deg": math.degrees(final_pose[2]),
                        "xy_error_cm": final_xy * 100.0,
                        "yaw_error_deg": final_yaw_err_deg,
                    }
            else:
                handoff_since = None

        # Timeout
        try:
            cancel_future = goal_handle.cancel_goal_async()
            self.wait_future(cancel_future, 3.0)
        except Exception:
            pass

        self.wait_robot_stopped(timeout_s=3.0)
        return False, {"reason": f"navigation timeout > {timeout_s:.1f}s"}


    def navigate_with_escape_recovery(
        self,
        name: str,
        cfg: Dict[str, float],
        bt_path: str,
        max_escape_retries: int = ESCAPE_MAX_RETRIES,
    ) -> Tuple[bool, Dict]:
        """
        FIX7 persistent recovery for recoverable navigation blockage.

        A recoverable failure is one of:
          - terminal anti-hunt guard
          - route blocked / no progress
          - Nav2 ABORTED status=6

        Recovery cycle:
          STOP/CANCEL -> hold -> verify fresh LiDAR
          -> try BackUp 0.25 m through behavior_server + normal safety chain
          -> clear local/global costmaps -> wait for fresh scan remarking
          -> resend the SAME NavigateToPose so Theta* replans.

        max_escape_retries == 0 means persistent/unlimited cycles. The mission
        still aborts on non-recoverable infrastructure/sensor failures or on
        operator SIGINT/SIGTERM. A blocked BackUp does NOT end the mission: the
        robot remains stopped, waits, refreshes costmaps and retries navigation.
        """
        attempts = []
        nav_attempt = 0
        recovery_cycle = 0

        while rclpy.ok():
            nav_attempt += 1
            ok, rec = self.navigate_to_station(name, cfg, bt_path)
            attempts.append({"nav_attempt": nav_attempt, **rec})

            if ok:
                if recovery_cycle == 0:
                    return True, rec
                merged = dict(rec)
                merged["escape_recovery_used"] = True
                merged["recovery_cycles"] = recovery_cycle
                merged["escape_attempts"] = attempts
                return True, merged

            recoverable_reasons = {
                "terminal anti-hunt guard triggered",
                "route blocked/no progress",
                "nav2 aborted status 6",
            }
            if rec.get("reason") not in recoverable_reasons:
                rec = dict(rec)
                rec["recovery_cycles"] = recovery_cycle
                rec["escape_attempts"] = attempts
                return False, rec

            recovery_cycle += 1
            if max_escape_retries > 0 and recovery_cycle > max_escape_retries:
                rec = dict(rec)
                rec["reason"] = "recoverable navigation blockage exceeded configured recovery limit"
                rec["recovery_cycles"] = recovery_cycle - 1
                rec["escape_attempts"] = attempts
                return False, rec

            print()
            print("#" * 68)
            limit_text = "unlimited" if max_escape_retries == 0 else str(max_escape_retries)
            print(
                f"PERSISTENT RECOVERY #{recovery_cycle} -> {name} "
                f"(configured limit={limit_text})"
            )
            print(f"Reason: {rec.get('reason')}")
            print(
                f"STOP {RECOVERY_STOP_HOLD_S:.2f}s -> try BackUp "
                f"{ESCAPE_BACKUP_M:.2f}m @ {ESCAPE_BACKUP_SPEED_MPS:.2f}m/s "
                "-> clear/rescan -> replan same goal"
            )
            print("Collision Monitor remains authoritative; unsafe reverse must be refused.")
            print("#" * 68)

            self.spin_for(RECOVERY_STOP_HOLD_S)

            scan_age = (
                time.monotonic() - self.last_scan_mono
                if self.last_scan_mono > 0.0
                else float("inf")
            )
            if scan_age > 0.40:
                return False, {
                    "reason": "persistent recovery refused: scan stale",
                    "scan_age_s": scan_age,
                    "recovery_cycles": recovery_cycle,
                    "escape_attempts": attempts,
                }

            bok, brec = self.backup(
                ESCAPE_BACKUP_M,
                ESCAPE_BACKUP_SPEED_MPS,
                f"ESCAPE_BACKUP_{name}_R{recovery_cycle}",
            )
            attempts.append({
                "stage": f"ESCAPE_BACKUP_{name}_R{recovery_cycle}",
                **brec,
            })

            # Infrastructure failures are not obstacle recovery cases.
            backup_reason = str(brec.get("reason", ""))
            infrastructure_backup_failure = backup_reason in {
                "backup server unavailable",
                "BackUp goal rejected",
                "BackUp timeout",
            }
            if not bok and infrastructure_backup_failure:
                return False, {
                    "reason": f"persistent recovery infrastructure failure: {backup_reason}",
                    "recovery_cycles": recovery_cycle,
                    "escape_attempts": attempts,
                }

            if not bok:
                print(
                    "RECOVERY BACKUP BLOCKED/ABORTED -> keep mission alive, "
                    f"wait {RECOVERY_BLOCKED_WAIT_S:.1f}s and refresh costmaps"
                )
                self.spin_for(RECOVERY_BLOCKED_WAIT_S)

            clear_rec = self.clear_costmaps_and_rescan(settle_s=0.80)
            attempts.append({
                "stage": f"ESCAPE_RESCAN_{name}_R{recovery_cycle}",
                **clear_rec,
            })
            if not clear_rec.get("scan_fresh_after_rescan", False):
                return False, {
                    "reason": "persistent recovery refused: scan not fresh after rescan",
                    "recovery_cycles": recovery_cycle,
                    "escape_attempts": attempts,
                }

            print(
                f"RECOVERY #{recovery_cycle} COMPLETE -> COSTMAP REFRESHED -> "
                f"REPLAN {name}"
            )

        return False, {
            "reason": "ROS shutdown during persistent recovery",
            "recovery_cycles": recovery_cycle,
            "escape_attempts": attempts,
        }

    # -------------------------------------------------------------------------
    # Exact-angle LiDAR swept-footprint safety check for turn_relative
    # -------------------------------------------------------------------------

    def _scan_points_in_base(
        self,
        scan: LaserScan,
    ) -> Optional[List[Tuple[float, float]]]:
        try:
            tr = self.tf_buffer.lookup_transform(
                BASE_FRAME,
                scan.header.frame_id,
                Time(),
            )
        except TransformException:
            return None

        q = tr.transform.rotation
        tf_yaw = yaw_from_quat(q)
        c = math.cos(tf_yaw)
        s = math.sin(tf_yaw)
        tx = float(tr.transform.translation.x)
        ty = float(tr.transform.translation.y)

        points = []
        a = float(scan.angle_min)

        for r in scan.ranges:
            rr = float(r)
            if (
                math.isfinite(rr)
                and float(scan.range_min) <= rr <= float(scan.range_max)
            ):
                xl = rr * math.cos(a)
                yl = rr * math.sin(a)

                xb = tx + c * xl - s * yl
                yb = ty + s * xl + c * yl
                points.append((xb, yb))
            a += float(scan.angle_increment)

        return points

    def _point_inside_footprint(self, x: float, y: float) -> bool:
        front, rear, left, right = self._profile_padded_bounds()
        return (
            -rear <= x <= front
            and -right <= y <= left
        )

    def swept_turn_safe(
        self,
        relative_angle_rad: float,
        scans_to_check: int = 5,
    ) -> Tuple[bool, Dict]:
        # Wait for fresh scan(s)
        scans = []
        last_stamp = None
        deadline = time.monotonic() + 4.0

        while rclpy.ok() and len(scans) < scans_to_check and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.08)
            scan = self.latest_scan
            if scan is None:
                continue

            stamp = (scan.header.stamp.sec, scan.header.stamp.nanosec)
            if stamp != last_stamp:
                scans.append(scan)
                last_stamp = stamp

        if not scans:
            return False, {"reason": "no scan for swept-turn safety"}

        requested_deg = math.degrees(relative_angle_rad)
        sign = 1.0 if relative_angle_rad >= 0.0 else -1.0
        sweep_deg = abs(requested_deg) + TURN_SWEEP_MARGIN_DEG

        steps = max(1, int(math.ceil(sweep_deg / TURN_SWEEP_STEP_DEG)))

        earliest_hit = None
        scans_ok = 0
        inside_now_max = 0

        for scan in scans:
            points = self._scan_points_in_base(scan)
            if points is None:
                continue
            scans_ok += 1

            inside_now = sum(
                1 for x, y in points if self._point_inside_footprint(x, y)
            )
            inside_now_max = max(inside_now_max, inside_now)

            if inside_now > 0:
                return False, {
                    "reason": "scan points already inside padded footprint",
                    "inside_now": inside_now,
                }

            for i in range(1, steps + 1):
                theta = sign * math.radians(
                    min(i * TURN_SWEEP_STEP_DEG, sweep_deg)
                )

                # Static world point expressed in the future robot frame:
                # p_future = R(-theta) * p_now
                c = math.cos(theta)
                s = math.sin(theta)

                hit = False
                for x, y in points:
                    xf = c * x + s * y
                    yf = -s * x + c * y

                    if self._point_inside_footprint(xf, yf):
                        hit = True
                        break

                if hit:
                    hit_deg = abs(math.degrees(theta))
                    if earliest_hit is None or hit_deg < earliest_hit:
                        earliest_hit = hit_deg
                    break

        if scans_ok == 0:
            return False, {"reason": "scan TF unavailable"}

        safe = earliest_hit is None
        return safe, {
            "requested_deg": requested_deg,
            "checked_deg": sweep_deg,
            "scans_checked": scans_ok,
            "inside_now_max": inside_now_max,
            "earliest_hit_deg": earliest_hit,
        }

    # -------------------------------------------------------------------------
    # One-shot yaw align using STM32 turn_relative
    # -------------------------------------------------------------------------

    def one_shot_yaw_align(
        self,
        station_name: str,
        cfg: Dict[str, float],
        retry_count: int = 0,
        turn_rate_override: Optional[float] = None,
    ) -> Tuple[bool, Dict]:
        pose = self.stable_map_pose()
        if pose is None:
            return False, {"reason": "map pose unavailable"}

        xy, err = pose_error(pose, cfg)
        err_deg = math.degrees(err)
        base_turn_rate = (
            TURN_RATE_A_H_RAD_S
            if station_name == "A_H"
            else TURN_RATE_DEFAULT_RAD_S
        )
        turn_rate = (
            float(turn_rate_override)
            if turn_rate_override is not None
            else base_turn_rate
        )

        print()
        print("-" * 68)
        print(f"ONE-SHOT YAW ALIGN -> {station_name}")
        print(
            f"CURRENT yaw={math.degrees(pose[2]):+.3f} deg | "
            f"TARGET={float(cfg['yaw_deg']):+.3f} deg"
        )
        print(
            f"XY ERROR={xy*100.0:.2f} cm | "
            f"YAW ERROR={err_deg:+.3f} deg"
        )

        rec = {
            "initial_x": pose[0],
            "initial_y": pose[1],
            "initial_yaw_deg": math.degrees(pose[2]),
            "initial_xy_error_cm": xy * 100.0,
            "initial_yaw_error_deg": err_deg,
            "yaw_retry_count": int(retry_count),
            "turn_rate_rad_s": float(turn_rate),
        }

        if abs(err_deg) < YAW_RELEASE_DEG:
            print(f"PASS_NO_TURN: |yaw error| < {YAW_RELEASE_DEG:.1f} deg")
            rec["result"] = "PASS_NO_TURN"
            return True, rec

        if abs(err_deg) > YAW_MAX_CORRECTION_DEG:
            print(
                f"REFUSE: |yaw error|={abs(err_deg):.3f} deg > "
                f"{YAW_MAX_CORRECTION_DEG:.1f} deg"
            )
            rec["reason"] = "yaw correction too large"
            return False, rec

        safe, safety = self.swept_turn_safe(err)
        rec["swept_safety"] = safety

        if not safe:
            print(f"REFUSE TURN: swept-area unsafe: {safety}")
            rec["reason"] = "swept-area unsafe"
            return False, rec

        print(
            f"SWEEP PASS: requested={err_deg:+.3f} deg | "
            f"checked={safety.get('checked_deg', 0):.2f} deg"
        )

        if self.turn_pub.get_subscription_count() < 1:
            self.spin_for(1.0)

        if self.turn_pub.get_subscription_count() < 1:
            rec["reason"] = "no subscriber on turn_relative"
            return False, rec

        timeout_s = min(
            15.0,
            max(
                1.0,
                abs(err) / turn_rate + TURN_EXTRA_TIMEOUT_S,
            ),
        )

        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = BASE_FRAME
        msg.vector.x = float(err)
        msg.vector.y = turn_rate
        msg.vector.z = timeout_s

        print(
            f"TURN_RELATIVE ONE SHOT: angle={err_deg:+.3f} deg | "
            f"rate<={turn_rate:.3f} rad/s | "
            f"timeout={timeout_s:.2f}s"
        )

        # Mark before publishing so Ctrl+C at any point after this line knows
        # an explicit firmware turn may still own the motors.
        self.turn_active_until_mono = (
            time.monotonic() + timeout_s + 2.5
        )
        self.turn_pub.publish(msg)

        # Allow publish/discovery and wait for STM32 turn to start/finish.
        self.spin_for(0.20)

        hard_deadline = time.monotonic() + timeout_s + 2.0
        motion_seen = False
        stopped_since = None

        while rclpy.ok() and time.monotonic() < hard_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            odom = self.latest_odom
            if odom is None:
                continue

            v = abs(float(odom.twist.twist.linear.x))
            w = abs(float(odom.twist.twist.angular.z))

            if w > 0.02:
                motion_seen = True
                stopped_since = None
            elif motion_seen and v < 0.01 and w < 0.012:
                if stopped_since is None:
                    stopped_since = time.monotonic()
                elif time.monotonic() - stopped_since >= 0.45:
                    break
            else:
                stopped_since = None

        self.wait_robot_stopped(timeout_s=2.0)
        self.turn_active_until_mono = 0.0

        final_pose = self.stable_map_pose()
        if final_pose is None:
            rec["reason"] = "final map pose unavailable"
            return False, rec

        final_xy, final_err = pose_error(final_pose, cfg)
        final_err_deg = math.degrees(final_err)

        rec.update(
            {
                "final_x": final_pose[0],
                "final_y": final_pose[1],
                "final_yaw_deg": math.degrees(final_pose[2]),
                "final_xy_error_cm": final_xy * 100.0,
                "final_yaw_error_deg": final_err_deg,
                "motion_seen": motion_seen,
            }
        )

        print(
            f"FINAL yaw={math.degrees(final_pose[2]):+.3f} deg | "
            f"FINAL ERROR={final_err_deg:+.3f} deg | "
            f"XY={final_xy*100.0:.2f} cm"
        )

        if abs(final_err_deg) < YAW_RELEASE_DEG:
            print(
                f"YAW RELEASE PASS: |error|={abs(final_err_deg):.3f} < "
                f"{YAW_RELEASE_DEG:.1f} deg"
            )
            rec["result"] = "PASS_ONE_SHOT"
            return True, rec

        # YAW_RETRY1: the original first correction remains unchanged.
        # Only if the measured final error is still inside the existing 8 deg
        # correction envelope do we allow ONE additional, re-measured turn.
        # The recursive call repeats the full swept-area safety check and is
        # explicitly prevented from retrying again.
        if (
            retry_count < YAW_FINE_RETRY_MAX
            and abs(final_err_deg) <= YAW_MAX_CORRECTION_DEG
        ):
            retry_rate = max(float(turn_rate), YAW_FINE_RETRY_RATE_RAD_S)
            print(
                f"YAW FINE RETRY {retry_count + 1}/{YAW_FINE_RETRY_MAX}: "
                f"remaining={final_err_deg:+.3f} deg | "
                f"rate<={retry_rate:.3f} rad/s"
            )
            ok_retry, retry_rec = self.one_shot_yaw_align(
                station_name,
                cfg,
                retry_count=retry_count + 1,
                turn_rate_override=retry_rate,
            )
            rec["fine_retry"] = retry_rec
            if ok_retry:
                rec["result"] = "PASS_AFTER_FINE_RETRY"
                return True, rec

            rec["reason"] = "final yaw outside release tolerance after bounded retry"
            return False, rec

        print("FAIL_YAW_FINAL: bounded yaw correction exhausted/refused")
        rec["reason"] = "final yaw outside release tolerance"
        return False, rec

    # -------------------------------------------------------------------------
    # DriveOnHeading / BackUp
    # -------------------------------------------------------------------------

    def _drive_feedback_cb(self, msg):
        try:
            self.last_drive_feedback = float(msg.feedback.distance_traveled)
        except Exception:
            pass

    def _backup_feedback_cb(self, msg):
        try:
            self.last_backup_feedback = float(msg.feedback.distance_traveled)
        except Exception:
            pass

    def drive_on_heading(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
    ) -> Tuple[bool, Dict]:
        print()
        print("-" * 68)
        print(
            f"DRIVE ON HEADING: {label} | "
            f"distance={distance_m:+.3f} m speed={speed_mps:.3f} m/s"
        )

        if not self.drive_client.wait_for_server(timeout_sec=5.0):
            return False, {"reason": "drive_on_heading server unavailable"}

        before = self.stable_map_pose()

        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance_m)
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(speed_mps)

        allowance = max(8.0, abs(distance_m) / max(speed_mps, 0.01) + 5.0)
        goal.time_allowance.sec = int(math.ceil(allowance))
        goal.time_allowance.nanosec = 0

        self.last_drive_feedback = None

        send_future = self.drive_client.send_goal_async(
            goal,
            feedback_callback=self._drive_feedback_cb,
        )
        goal_handle = self.wait_future(send_future, 8.0)

        if goal_handle is None or not goal_handle.accepted:
            return False, {"reason": "DriveOnHeading goal rejected"}

        t0 = time.monotonic()
        result_future = goal_handle.get_result_async()
        wrapped = self.wait_future(result_future, allowance + 8.0)
        elapsed = time.monotonic() - t0

        if wrapped is None:
            try:
                self.wait_future(goal_handle.cancel_goal_async(), 3.0)
            except Exception:
                pass
            return False, {"reason": "DriveOnHeading timeout"}

        self.wait_robot_stopped(timeout_s=3.0)
        after = self.stable_map_pose()

        status = int(wrapped.status)
        rec = {
            "status": status,
            "requested_distance_m": distance_m,
            "requested_speed_mps": speed_mps,
            "feedback_distance_m": self.last_drive_feedback,
            "elapsed_s": elapsed,
        }

        if before is not None:
            rec["before"] = {
                "x": before[0],
                "y": before[1],
                "yaw_deg": math.degrees(before[2]),
            }

        if after is not None:
            rec["after"] = {
                "x": after[0],
                "y": after[1],
                "yaw_deg": math.degrees(after[2]),
            }

        if before is not None and after is not None:
            dx = after[0] - before[0]
            dy = after[1] - before[1]

            c = math.cos(before[2])
            s = math.sin(before[2])

            fwd = c * dx + s * dy
            lat = -s * dx + c * dy
            yaw_delta = math.degrees(wrap_pi(after[2] - before[2]))

            rec.update(
                {
                    "map_fwd_m": fwd,
                    "map_lat_m": lat,
                    "map_yaw_delta_deg": yaw_delta,
                }
            )

            print(
                f"MAP DELTA: fwd={fwd:+.3f} m | lat={lat:+.3f} m | "
                f"dyaw={yaw_delta:+.3f} deg"
            )

        print(
            f"RESULT status={status} | feedback_distance="
            f"{self.last_drive_feedback} | elapsed={elapsed:.2f}s"
        )

        return status == GoalStatus.STATUS_SUCCEEDED, rec

    def drive_straight_heading_lock(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
    ) -> Tuple[bool, Dict]:
        """FIX10 yaw-hold controller retained by FIX11 for the TH drop approach.

        This is intentionally used ONLY for the TH drop approach. It does not bypass
        velocity_smoother, turn gate, collision monitor, or /cmd_vel_safe.
        """
        print()
        print("-" * 68)
        print(
            f"ODOM HEADING LOCK: {label} | "
            f"distance={distance_m:+.3f} m speed={speed_mps:.3f} m/s"
        )

        if distance_m <= 0.0:
            return False, {"reason": "heading-lock only supports positive distance"}
        if speed_mps <= 0.0:
            return False, {"reason": "heading-lock speed must be positive"}
        if not self.wait_robot_stopped(timeout_s=3.0):
            return False, {"reason": "robot not stopped before heading-lock"}

        self.spin_for(0.35)
        now = time.monotonic()
        if self.latest_odom is None or now - self.last_odom_mono > TH_LOCK_ODOM_MAX_AGE_S:
            return False, {"reason": "stale/missing odom before heading-lock"}
        if self.latest_scan is None or now - self.last_scan_mono > TH_LOCK_SCAN_MAX_AGE_S:
            return False, {"reason": "stale/missing scan before heading-lock"}

        od = self.latest_odom
        start_x = float(od.pose.pose.position.x)
        start_y = float(od.pose.pose.position.y)
        start_yaw = yaw_from_quat(od.pose.pose.orientation)
        before_map = self.stable_map_pose(samples=8, timeout_s=2.5)

        pub = self.create_publisher(Twist, CMD_NAV, 10)
        period = 1.0 / TH_LOCK_CONTROL_HZ
        allowance = max(10.0, distance_m / max(speed_mps, 0.01) + 7.0)
        t0 = time.monotonic()
        last_progress_t = t0
        best_fwd = 0.0
        max_abs_yaw_err = 0.0
        max_abs_lat = 0.0
        last_log_t = 0.0
        success = False
        reason = None
        final_odom = None

        try:
            while rclpy.ok() and time.monotonic() - t0 < allowance:
                loop_t = time.monotonic()
                rclpy.spin_once(self, timeout_sec=0.01)

                now = time.monotonic()
                if self.latest_odom is None or now - self.last_odom_mono > TH_LOCK_ODOM_MAX_AGE_S:
                    reason = "odom stale during heading-lock"
                    break
                if self.latest_scan is None or now - self.last_scan_mono > TH_LOCK_SCAN_MAX_AGE_S:
                    reason = "scan stale during heading-lock"
                    break

                od = self.latest_odom
                x = float(od.pose.pose.position.x)
                y = float(od.pose.pose.position.y)
                yaw = yaw_from_quat(od.pose.pose.orientation)

                dx = x - start_x
                dy = y - start_y
                c0 = math.cos(start_yaw)
                s0 = math.sin(start_yaw)
                fwd = c0 * dx + s0 * dy
                lat = -s0 * dx + c0 * dy
                yaw_err = wrap_pi(yaw - start_yaw)
                remaining = distance_m - fwd

                max_abs_yaw_err = max(max_abs_yaw_err, abs(yaw_err))
                max_abs_lat = max(max_abs_lat, abs(lat))
                final_odom = {
                    "x": x,
                    "y": y,
                    "yaw_deg": math.degrees(yaw),
                    "fwd_m": fwd,
                    "lat_m": lat,
                    "yaw_error_deg": math.degrees(yaw_err),
                }

                if fwd >= distance_m:
                    success = True
                    break
                if abs(math.degrees(yaw_err)) > TH_LOCK_ABORT_YAW_DEG:
                    reason = f"heading-lock yaw exceeded {TH_LOCK_ABORT_YAW_DEG:.1f} deg"
                    break
                if abs(lat) > TH_LOCK_ABORT_LAT_M:
                    reason = f"heading-lock lateral exceeded {TH_LOCK_ABORT_LAT_M:.3f} m"
                    break

                if fwd > best_fwd + 0.005:
                    best_fwd = fwd
                    last_progress_t = now
                elif now - last_progress_t > TH_LOCK_STALL_S:
                    reason = "heading-lock stalled/blocked"
                    break

                if remaining > TH_LOCK_SLOWDOWN_M:
                    v_cmd = float(speed_mps)
                else:
                    scaled = float(speed_mps) * max(remaining, 0.0) / TH_LOCK_SLOWDOWN_M
                    v_cmd = max(TH_LOCK_MIN_V_MPS, min(float(speed_mps), scaled))

                w_cmd = -TH_LOCK_K_YAW * yaw_err
                w_cmd = max(-TH_LOCK_MAX_W_RAD_S, min(TH_LOCK_MAX_W_RAD_S, w_cmd))

                cmd = Twist()
                cmd.linear.x = float(v_cmd)
                cmd.angular.z = float(w_cmd)
                pub.publish(cmd)

                if now - last_log_t >= 0.25:
                    print(
                        f"TH LOCK fwd={fwd:.3f}/{distance_m:.3f}m "
                        f"lat={lat*100.0:+.1f}cm "
                        f"yaw_err={math.degrees(yaw_err):+.2f}deg "
                        f"v={v_cmd:.3f} w={w_cmd:+.3f}"
                    )
                    last_log_t = now

                spent = time.monotonic() - loop_t
                if spent < period:
                    time.sleep(period - spent)

            if not success and reason is None:
                reason = "heading-lock timeout/ROS shutdown"

        finally:
            zero = Twist()
            for _ in range(8):
                pub.publish(zero)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.04)
            try:
                self.destroy_publisher(pub)
            except Exception:
                pass

        stopped = self.wait_robot_stopped(timeout_s=3.0)
        after_map = self.stable_map_pose(samples=8, timeout_s=2.5)
        elapsed = time.monotonic() - t0

        rec = {
            "result": "SUCCEEDED" if success and stopped else "FAILED",
            "controller": "ODOM_YAW_HOLD_VIA_CMD_VEL_NAV",
            "requested_distance_m": float(distance_m),
            "requested_speed_mps": float(speed_mps),
            "elapsed_s": elapsed,
            "start_odom": {
                "x": start_x,
                "y": start_y,
                "yaw_deg": math.degrees(start_yaw),
            },
            "final_odom": final_odom,
            "max_abs_yaw_error_deg": math.degrees(max_abs_yaw_err),
            "max_abs_lateral_m": max_abs_lat,
            "robot_stopped": bool(stopped),
        }
        if reason is not None:
            rec["reason"] = reason

        if before_map is not None:
            rec["before"] = {
                "x": before_map[0],
                "y": before_map[1],
                "yaw_deg": math.degrees(before_map[2]),
            }
        if after_map is not None:
            rec["after"] = {
                "x": after_map[0],
                "y": after_map[1],
                "yaw_deg": math.degrees(after_map[2]),
            }
        if before_map is not None and after_map is not None:
            dx = after_map[0] - before_map[0]
            dy = after_map[1] - before_map[1]
            c = math.cos(before_map[2])
            ss = math.sin(before_map[2])
            rec["map_fwd_m"] = c * dx + ss * dy
            rec["map_lat_m"] = -ss * dx + c * dy
            rec["map_yaw_delta_deg"] = math.degrees(
                wrap_pi(after_map[2] - before_map[2])
            )
            print(
                f"TH LOCK MAP DELTA: fwd={rec['map_fwd_m']:+.3f} m | "
                f"lat={rec['map_lat_m']:+.3f} m | "
                f"dyaw={rec['map_yaw_delta_deg']:+.3f} deg"
            )

        print(
            f"TH LOCK RESULT: {'PASS' if success and stopped else 'FAIL'} | "
            f"elapsed={elapsed:.2f}s | reason={reason}"
        )
        return bool(success and stopped), rec

    def backup(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
    ) -> Tuple[bool, Dict]:
        print()
        print("-" * 68)
        print(
            f"BACKUP: {label} | "
            f"distance={distance_m:.3f} m speed={speed_mps:.3f} m/s"
        )

        if not self.backup_client.wait_for_server(timeout_sec=5.0):
            return False, {"reason": "backup server unavailable"}

        before = self.stable_map_pose()

        goal = BackUp.Goal()
        # Direct nav2_msgs/BackUp action: negative X is reverse in base_link.
        goal.target.x = -float(abs(distance_m))
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = float(abs(speed_mps))

        allowance = max(8.0, abs(distance_m) / max(speed_mps, 0.01) + 5.0)
        goal.time_allowance.sec = int(math.ceil(allowance))
        goal.time_allowance.nanosec = 0

        self.last_backup_feedback = None

        send_future = self.backup_client.send_goal_async(
            goal,
            feedback_callback=self._backup_feedback_cb,
        )
        goal_handle = self.wait_future(send_future, 8.0)

        if goal_handle is None or not goal_handle.accepted:
            return False, {"reason": "BackUp goal rejected"}

        t0 = time.monotonic()
        result_future = goal_handle.get_result_async()
        wrapped = self.wait_future(result_future, allowance + 8.0)
        elapsed = time.monotonic() - t0

        if wrapped is None:
            try:
                self.wait_future(goal_handle.cancel_goal_async(), 3.0)
            except Exception:
                pass
            return False, {"reason": "BackUp timeout"}

        self.wait_robot_stopped(timeout_s=3.0)
        after = self.stable_map_pose()

        status = int(wrapped.status)
        rec = {
            "status": status,
            "requested_distance_m": distance_m,
            "requested_speed_mps": speed_mps,
            "feedback_distance_m": self.last_backup_feedback,
            "elapsed_s": elapsed,
        }

        if before is not None:
            rec["before"] = {
                "x": before[0],
                "y": before[1],
                "yaw_deg": math.degrees(before[2]),
            }

        if after is not None:
            rec["after"] = {
                "x": after[0],
                "y": after[1],
                "yaw_deg": math.degrees(after[2]),
            }

        if before is not None and after is not None:
            dx = after[0] - before[0]
            dy = after[1] - before[1]

            c = math.cos(before[2])
            s = math.sin(before[2])

            fwd = c * dx + s * dy
            lat = -s * dx + c * dy
            yaw_delta = math.degrees(wrap_pi(after[2] - before[2]))

            rec.update(
                {
                    "map_fwd_m": fwd,
                    "map_lat_m": lat,
                    "map_yaw_delta_deg": yaw_delta,
                }
            )

            print(
                f"MAP DELTA: fwd={fwd:+.3f} m | lat={lat:+.3f} m | "
                f"dyaw={yaw_delta:+.3f} deg"
            )

        print(
            f"RESULT status={status} | feedback_distance="
            f"{self.last_backup_feedback} | elapsed={elapsed:.2f}s"
        )

        return status == GoalStatus.STATUS_SUCCEEDED, rec


    @staticmethod
    def _motion_progress_from_record(rec: Dict, direction_sign: float) -> float:
        """Best-effort progress in the commanded direction, in meters."""
        fb = rec.get("feedback_distance_m")
        try:
            if fb is not None and math.isfinite(float(fb)):
                return max(0.0, abs(float(fb)))
        except Exception:
            pass

        try:
            fwd = float(rec.get("map_fwd_m", 0.0))
            return max(0.0, direction_sign * fwd)
        except Exception:
            return 0.0

    def drive_on_heading_persistent(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
        max_attempts: int = MOTION_RETRY_MAX_ATTEMPTS,
    ) -> Tuple[bool, Dict]:
        """
        Retry a fixed DriveOnHeading motion after recoverable collision aborts.

        The behavior's own collision checking and the downstream Collision Monitor
        remain enabled. A retry never disables collision checks. Progress from a
        partial attempt is subtracted so the requested TOTAL distance is preserved.
        max_attempts=0 means persistent while /scan stays healthy.
        """
        sign = 1.0 if distance_m >= 0.0 else -1.0
        remaining = abs(float(distance_m))
        attempts = []
        attempt = 0

        while rclpy.ok() and remaining > 0.02:
            attempt += 1
            ok, rec = self.drive_on_heading(sign * remaining, speed_mps, f"{label}_TRY{attempt}")
            attempts.append({"attempt": attempt, **rec})

            progress = min(remaining, self._motion_progress_from_record(rec, sign))
            remaining = max(0.0, remaining - progress)

            if ok or remaining <= 0.02:
                return True, {
                    "result": "SUCCEEDED",
                    "requested_total_distance_m": float(distance_m),
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            reason = str(rec.get("reason", ""))
            if reason in {
                "drive_on_heading server unavailable",
                "DriveOnHeading goal rejected",
                "DriveOnHeading timeout",
            }:
                return False, {
                    "reason": reason,
                    "requested_total_distance_m": float(distance_m),
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            if max_attempts > 0 and attempt >= max_attempts:
                return False, {
                    "reason": "DriveOnHeading recoverable retries exhausted",
                    "requested_total_distance_m": float(distance_m),
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            stop_rec = self.safe_stop_all(
                f"MOTION_RETRY_{label}_TRY{attempt}",
                wait_stop_s=3.0,
            )
            attempts[-1]["safe_stop"] = stop_rec

            scan_age = (
                time.monotonic() - self.last_scan_mono
                if self.last_scan_mono > 0.0 else float("inf")
            )
            if scan_age > 0.40:
                return False, {
                    "reason": "DriveOnHeading retry refused: scan stale",
                    "scan_age_s": scan_age,
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            print(
                f"DRIVE RETRY: {label} remaining={remaining:.3f}m | "
                f"wait {RECOVERY_BLOCKED_WAIT_S:.1f}s -> clear/rescan -> retry"
            )
            self.spin_for(RECOVERY_BLOCKED_WAIT_S)
            clear_rec = self.clear_costmaps_and_rescan(settle_s=0.80)
            attempts[-1]["clear_rescan"] = clear_rec
            if not clear_rec.get("scan_fresh_after_rescan", False):
                return False, {
                    "reason": "DriveOnHeading retry refused: scan not fresh after rescan",
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

        return False, {
            "reason": "ROS shutdown during DriveOnHeading persistent retry",
            "remaining_distance_m": remaining,
            "attempt_count": attempt,
            "attempts": attempts,
        }

    def backup_persistent(
        self,
        distance_m: float,
        speed_mps: float,
        label: str,
        max_attempts: int = MOTION_RETRY_MAX_ATTEMPTS,
    ) -> Tuple[bool, Dict]:
        """
        Retry a fixed reverse motion without disabling collision checks.

        Partial reverse progress is subtracted, so a requested 2.00 m exit remains
        2.00 m total rather than 2.00 m on every retry. If the rear is blocked,
        the robot stays stopped, waits for fresh sensor data, clears/rescans, and
        retries. Operator SIGINT/SIGTERM still performs SAFE STOP.
        """
        remaining = abs(float(distance_m))
        attempts = []
        attempt = 0

        while rclpy.ok() and remaining > 0.02:
            attempt += 1
            ok, rec = self.backup(remaining, speed_mps, f"{label}_TRY{attempt}")
            attempts.append({"attempt": attempt, **rec})

            progress = min(remaining, self._motion_progress_from_record(rec, -1.0))
            remaining = max(0.0, remaining - progress)

            if ok or remaining <= 0.02:
                return True, {
                    "result": "SUCCEEDED",
                    "requested_total_distance_m": abs(float(distance_m)),
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            reason = str(rec.get("reason", ""))
            if reason in {
                "backup server unavailable",
                "BackUp goal rejected",
                "BackUp timeout",
            }:
                return False, {
                    "reason": reason,
                    "requested_total_distance_m": abs(float(distance_m)),
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            if max_attempts > 0 and attempt >= max_attempts:
                return False, {
                    "reason": "BackUp recoverable retries exhausted",
                    "requested_total_distance_m": abs(float(distance_m)),
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            stop_rec = self.safe_stop_all(
                f"BACKUP_RETRY_{label}_TRY{attempt}",
                wait_stop_s=3.0,
            )
            attempts[-1]["safe_stop"] = stop_rec

            scan_age = (
                time.monotonic() - self.last_scan_mono
                if self.last_scan_mono > 0.0 else float("inf")
            )
            if scan_age > 0.40:
                return False, {
                    "reason": "BackUp retry refused: scan stale",
                    "scan_age_s": scan_age,
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

            print(
                f"BACKUP RETRY: {label} remaining={remaining:.3f}m | rear blocked/abort -> "
                f"wait {RECOVERY_BLOCKED_WAIT_S:.1f}s -> clear/rescan -> retry"
            )
            self.spin_for(RECOVERY_BLOCKED_WAIT_S)
            clear_rec = self.clear_costmaps_and_rescan(settle_s=0.80)
            attempts[-1]["clear_rescan"] = clear_rec
            if not clear_rec.get("scan_fresh_after_rescan", False):
                return False, {
                    "reason": "BackUp retry refused: scan not fresh after rescan",
                    "remaining_distance_m": remaining,
                    "attempt_count": attempt,
                    "attempts": attempts,
                }

        return False, {
            "reason": "ROS shutdown during BackUp persistent retry",
            "remaining_distance_m": remaining,
            "attempt_count": attempt,
            "attempts": attempts,
        }


# =============================================================================
# FILES / BAG / SUMMARY
# =============================================================================

def load_stations(path: str) -> Dict[str, Dict[str, float]]:
    p = Path(path).expanduser()
    if not p.is_file():
        raise FileNotFoundError(f"stations file not found: {p}")

    data = yaml.safe_load(p.read_text())

    if not isinstance(data, dict):
        raise ValueError("station YAML root must be a dict")

    required = ["A_H", "B1", "B2", "TH"]

    if all(name in data for name in required):
        stations = data
    else:
        stations = None
        for key, value in data.items():
            if isinstance(value, dict) and all(name in value for name in required):
                stations = value
                print(f"STATION YAML WRAPPER DETECTED: {key}")
                break

        if stations is None:
            raise KeyError(
                "could not find A_H, B1, B2, TH at root or inside a YAML wrapper"
            )

    for name in required:
        cfg = stations[name]
        if not isinstance(cfg, dict):
            raise ValueError(f"{name} must be a mapping")
        for key in ("x", "y", "yaw_deg"):
            if key not in cfg:
                raise KeyError(f"{name} missing key: {key}")

    return stations



def write_temp_bt() -> str:
    path = Path(tempfile.gettempdir()) / "amr_demo_fast_v3.xml"

    # Fail-fast before any NavigateToPose goal is sent.
    try:
        ET.fromstring(RECOVERY_BT_XML)
    except ET.ParseError as exc:
        raise RuntimeError(f"INVALID RECOVERY BT XML: {exc}") from exc

    path.write_text(RECOVERY_BT_XML)
    return str(path)


def available_topics() -> set:
    try:
        out = subprocess.check_output(
            ["ros2", "topic", "list"],
            text=True,
            stderr=subprocess.DEVNULL,
            timeout=5.0,
        )
        return {line.strip() for line in out.splitlines() if line.strip()}
    except Exception:
        return set()


def start_bag_recording(output_dir: Path):
    topics = available_topics()
    selected = [t for t in BAG_TOPIC_CANDIDATES if t in topics]

    if not selected:
        print("BAG: no selected topics available; bag recording skipped")
        return None, None, []

    bag_dir = output_dir / "bag"
    recorder_log = output_dir / "rosbag_recorder.log"

    f = recorder_log.open("w", encoding="utf-8")

    cmd = [
        "ros2",
        "bag",
        "record",
        "-o",
        str(bag_dir),
        *selected,
    ]

    try:
        proc = subprocess.Popen(
            cmd,
            stdout=f,
            stderr=subprocess.STDOUT,
            preexec_fn=os.setsid,
        )
        print(f"BAG: recording {len(selected)} topics -> {bag_dir}")
        return proc, f, selected
    except Exception as exc:
        f.close()
        print(f"BAG START WARNING: {exc}")
        return None, None, []


def stop_bag_recording(proc, log_file):
    if proc is not None and proc.poll() is None:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10.0)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
            except Exception:
                pass

    if log_file is not None:
        try:
            log_file.close()
        except Exception:
            pass


def json_safe(value):
    if isinstance(value, dict):
        return {str(k): json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [json_safe(v) for v in value]
    if isinstance(value, tuple):
        return [json_safe(v) for v in value]
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    return str(value)


def ros2_param_get(node_name: str, param_name: str) -> str:
    out = subprocess.check_output(
        ["ros2", "param", "get", node_name, param_name],
        text=True,
        stderr=subprocess.STDOUT,
        timeout=15.0,
    )
    return out.strip()


def parse_ros2_double(output: str) -> float:
    m = re.search(r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*$", output)
    if not m:
        raise ValueError(f"cannot parse double from: {output!r}")
    return float(m.group(1))


def parse_ros2_double_array(output: str) -> List[float]:
    m = re.search(r"\[([^\]]+)\]", output)
    if not m:
        raise ValueError(f"cannot parse double array from: {output!r}")
    vals = []
    for part in m.group(1).split(","):
        vals.append(float(part.strip()))
    return vals


def parse_ros2_bool(output: str) -> bool:
    text = output.strip().lower()
    if text.endswith("true"):
        return True
    if text.endswith("false"):
        return False
    raise ValueError(f"cannot parse bool from: {output!r}")


# =============================================================================
# MISSION
# =============================================================================

def parse_args():
    p = argparse.ArgumentParser(
        description=(
            "Run standalone FIX11 B2 QR: A_H -> B2 -> QR gate -> OPEN -> "
            "+1.50m -> PICK -> -2.00m -> TH -> +0.40m heading-lock -> "
            "DROP -> -0.40m -> CLOSE -> A_H."
        )
    )
    p.add_argument(
        "--stations",
        default=DEFAULT_STATIONS,
        help="station YAML path",
    )
    p.add_argument(
        "--pickup-speed",
        type=float,
        default=DEFAULT_PICKUP_SPEED_MPS,
        help=f"forward pickup speed m/s (default {DEFAULT_PICKUP_SPEED_MPS:.2f}, max {MAX_PICKUP_SPEED_MPS:.2f})",
    )
    p.add_argument(
        "--backup-speed",
        type=float,
        default=DEFAULT_BACKUP_SPEED_MPS,
        help=f"pickup-exit reverse speed m/s (default {DEFAULT_BACKUP_SPEED_MPS:.2f}, max {MAX_BACKUP_SPEED_MPS:.2f})",
    )
    p.add_argument(
        "--skip-home-check",
        action="store_true",
        help="allow starting away from A_H (NOT recommended for acceptance)",
    )
    p.add_argument(
        "--no-bag",
        action="store_true",
        help="disable automatic rosbag recording",
    )
    p.add_argument(
        "--preflight-only",
        action="store_true",
        help="run all no-motion preflight checks, including actuator + QR camera identity, then exit",
    )
    p.add_argument(
        "--qr-test-only",
        action="store_true",
        help="after no-motion preflight, scan QR only; NO motion goal is sent",
    )
    return p.parse_args()


def main() -> int:
    args = parse_args()
    route = FIX8_ROUTE
    pickup_distance_m = FIX8_PICKUP_DISTANCE_M
    exit_backup_distance_m = FIX8_PICK_EXIT_BACKUP_M

    if os.environ.get("ROS_DOMAIN_ID") != "42":
        print("FAIL: ROS_DOMAIN_ID must be 42")
        print("Run: export ROS_DOMAIN_ID=42")
        return 2

    if not (0.01 <= args.pickup_speed <= MAX_PICKUP_SPEED_MPS):
        print(f"FAIL CONFIG: pickup speed must be 0.01..{MAX_PICKUP_SPEED_MPS:.2f} m/s")
        return 2
    if not (0.01 <= args.backup_speed <= MAX_BACKUP_SPEED_MPS):
        print(f"FAIL CONFIG: backup speed must be 0.01..{MAX_BACKUP_SPEED_MPS:.2f} m/s")
        return 2

    # B1 and B2 standalone files share one process lock so they cannot run together.
    lock_fh = open(MISSION_LOCK_PATH, "w")
    try:
        fcntl.flock(lock_fh.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        print(f"FAIL: another FIX8 B1/B2 mission already owns {MISSION_LOCK_PATH}")
        return 2
    lock_fh.write(f"pid={os.getpid()} route={route}\n")
    lock_fh.flush()

    try:
        stations = load_stations(args.stations)
    except Exception as exc:
        print(f"FAIL STATIONS: {exc}")
        return 2

    bt_path = write_temp_bt()

    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    output_dir = Path.home() / "Downloads" / f"AMR_DEMO_FAST_V3_FIX11_{route}_QR_{timestamp}"
    output_dir.mkdir(parents=True, exist_ok=True)

    summary = {
        "mission": "A_H -> B2 -> QR_GATE(BOX_001) -> OPEN -> PICK_FORWARD -> PICK -> PICK_EXIT_BACKUP -> TH -> TH+50 -> DROP -> TH-50 -> CLOSE -> A_H",
        "version": "AMR_DEMO_FAST_V3_FIX11_B2_QR",
        "timestamp": timestamp,
        "route": route,
        "pickup_distance_m": pickup_distance_m,
        "pickup_speed_mps": float(args.pickup_speed),
        "pickup_exit_backup_distance_m": exit_backup_distance_m,
        "backup_speed_mps": float(args.backup_speed),
        "stations_file": str(Path(args.stations).expanduser()),
        "bt_path_runtime": bt_path,
        "policy": {
            "station_xy_handoff_m": STATION_HANDOFF_XY_M,
            "nav_yaw_handoff_deg": NAV_YAW_HANDOFF_DEG,
            "yaw_release_deg": YAW_RELEASE_DEG,
            "automatic_recovery_backup": True,
            "automatic_recovery_spin": False,
            "recovery": "recoverable block/status6 -> stop -> safe backup 0.25m -> clear/rescan -> replan persistently; backup-blocked -> wait/clear/replan",
            "empty_footprint_m": {"front": EMPTY_FRONT_M, "rear": EMPTY_REAR_M, "width": EMPTY_LEFT_M + EMPTY_RIGHT_M},
            "loaded_footprint_m": {"front": LOADED_FRONT_M, "rear": LOADED_REAR_M, "width": LOADED_LEFT_M + LOADED_RIGHT_M},
            "qr_gate": {
                "camera_by_id": QR_CAMERA_BY_ID,
                "expected": QR_EXPECTED_B2,
                "required_hits": QR_REQUIRED_HITS,
                "window_s": QR_WINDOW_S,
                "timeout_s": QR_TIMEOUT_S,
                "camera_role": "AUTH_ONLY_NO_YAW_NO_STEERING",
            },
            "pickup_actuator": "STM32_V2_12_OPEN_PICK_WAIT_PICK_DONE",
            "drop_actuator": "STM32_V2_12_DROP_WAIT_DROP_DONE_THEN_ARM_CLOSE_CLOSE",
            "actuator_port_policy": "AUTO_DISCOVER_NON_DRIVE_STM32_THEN_VERIFY_EXACT_VERSION",
            "drive_stm32_by_id_excluded": DRIVE_STM32_BY_ID,
            "actuator_expected_version": ACTUATOR_EXPECTED_VERSION,
            "th_drop_forward_controller": "ODOM_YAW_HOLD_VIA_CMD_VEL_NAV",
        },
        "stages": [],
        "success": False,
        "failed_stage": None,
    }

    bag_proc = None
    bag_log = None

    if SignalHandlerOptions is not None:
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    else:
        rclpy.init()

    node = DemoFastNode()
    actuator = ActuatorClient()

    def _mission_signal_handler(signum, _frame):
        signame = "SIGINT" if signum == signal.SIGINT else "SIGTERM"
        print(f"\n{signame}: requesting SAFE mission abort")
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _mission_signal_handler)
    signal.signal(signal.SIGTERM, _mission_signal_handler)

    finish_called = False

    def finish(success: bool, failed_stage: Optional[str] = None) -> int:
        nonlocal finish_called
        finish_called = True
        summary["success"] = bool(success)
        summary["failed_stage"] = failed_stage
        summary["result"] = "MISSION_PASS" if success else "MISSION_FAIL"

        # Cancel all ROS motion authority before stopping bag / destroying node.
        safe_stop = node.safe_stop_all(
            "MISSION_FINISH" if success else f"MISSION_ABORT_{failed_stage}",
            wait_stop_s=4.0,
        )
        summary["safe_stop"] = safe_stop

        if not success:
            actuator.safe_stop()
        actuator.close()

        summary_path = output_dir / "mission_summary.json"
        summary_path.write_text(
            json.dumps(json_safe(summary), indent=2, ensure_ascii=False)
        )
        stop_bag_recording(bag_proc, bag_log)

        print()
        print("=" * 72)
        if success:
            print(f"MISSION PASS: A_H -> {route} -> PICK -> TH/DROP/CLOSE -> A_H COMPLETED; READY FOR NEXT TASK")
        else:
            print(f"MISSION STOPPED AT STAGE: {failed_stage}")
        print(f"SUMMARY: {summary_path}")
        if not args.no_bag:
            print(f"BAG DIR : {output_dir / 'bag'}")
        print("=" * 72)
        return 0 if success else 3

    def finish_qr_reject(qr_result: str) -> int:
        nonlocal finish_called
        finish_called = True
        summary["success"] = True
        summary["failed_stage"] = None
        summary["result"] = "MISSION_QR_REJECT_RETURNED_HOME"
        summary["qr_reject_reason"] = qr_result
        summary["safe_stop"] = node.safe_stop_all("MISSION_QR_REJECT_FINISH", wait_stop_s=4.0)
        actuator.close()
        summary_path = output_dir / "mission_summary.json"
        summary_path.write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False))
        stop_bag_recording(bag_proc, bag_log)
        print()
        print("=" * 72)
        print(f"MISSION COMPLETE: {qr_result}; NO PICK; RETURNED A_H; READY FOR NEXT TASK")
        print(f"SUMMARY: {summary_path}")
        if not args.no_bag:
            print(f"BAG DIR : {output_dir / 'bag'}")
        print("=" * 72)
        return 0

    try:
        print()
        print("=" * 72)
        print("AMR DEMO FAST V3 FIX11 B2 QR")
        print("ROUTE: A_H -> B2 -> QR(BOX_001) -> OPEN/PICK -> TH/LOCK+0.40/DROP/CLOSE -> A_H")
        print("Theta* + MPPI | NAV target 0.35 m/s | TEST footprint fixed before/after PICK")
        print("NO AUTO SPIN | BLOCKED/STATUS6 -> PERSISTENT STOP/BACKUP/RESCAN/REPLAN | SAFETY CHAIN REQUIRED")
        print(
            f"PICK: +{pickup_distance_m:.2f} m @ {args.pickup_speed:.2f} m/s | "
            f"EXIT: -{exit_backup_distance_m:.2f} m @ {args.backup_speed:.2f} m/s"
        )
        print("=" * 72)

        # ------------------------------------------------------------------
        # PREFLIGHT: graph + lifecycle + controller identity + safety chain
        # ------------------------------------------------------------------
        ok_nodes, missing = node.wait_required_nodes()
        if not ok_nodes:
            summary["preflight"] = {"missing_nodes": missing}
            print("FAIL PREFLIGHT: missing required nodes:")
            for n in missing:
                print(f"  {n}")
            return finish(False, "PREFLIGHT_NODES")

        cm_active = node.lifecycle_active("/collision_monitor")
        vs_active = node.lifecycle_active("/velocity_smoother")
        cs_active = node.lifecycle_active("/controller_server")
        ps_active = node.lifecycle_active("/planner_server")
        bt_active = node.lifecycle_active("/bt_navigator")

        try:
            plugin_out = ros2_param_get("/controller_server", "FollowPath.plugin")
            vx_out = ros2_param_get("/controller_server", "FollowPath.vx_max")
            vx_max = parse_ros2_double(vx_out)
            smoother_out = ros2_param_get("/velocity_smoother", "max_velocity")
            smoother_max = parse_ros2_double_array(smoother_out)
            smoother_min_out = ros2_param_get("/velocity_smoother", "min_velocity")
            smoother_min = parse_ros2_double_array(smoother_min_out)
            smoother_accel_out = ros2_param_get("/velocity_smoother", "max_accel")
            smoother_accel = parse_ros2_double_array(smoother_accel_out)
            smoother_decel_out = ros2_param_get("/velocity_smoother", "max_decel")
            smoother_decel = parse_ros2_double_array(smoother_decel_out)
            cm_points_out = ros2_param_get("/collision_monitor", "BodyEmergencyStop.points")
            cm_points = parse_ros2_double_array(cm_points_out)
            cm_fp_topic_out = ros2_param_get("/collision_monitor", "FootprintApproach.footprint_topic")
            cm_ttc_out = ros2_param_get("/collision_monitor", "FootprintApproach.time_before_collision")
            cm_ttc = parse_ros2_double(cm_ttc_out)
            local_inf_out = ros2_param_get(
                "/local_costmap/local_costmap", "obstacle_layer.scan.inf_is_valid"
            )
            global_inf_out = ros2_param_get(
                "/global_costmap/global_costmap", "obstacle_layer.scan.inf_is_valid"
            )
            local_plugins_out = ros2_param_get(
                "/local_costmap/local_costmap", "plugins"
            )
            local_inf_valid = parse_ros2_bool(local_inf_out)
            global_inf_valid = parse_ros2_bool(global_inf_out)
        except Exception as exc:
            print(f"FAIL PREFLIGHT: controller parameter audit failed: {exc}")
            return finish(False, "PREFLIGHT_CONTROLLER_PARAMS")

        mppi_ok = "nav2_mppi_controller::MPPIController" in plugin_out
        vx_ok = abs(vx_max - 0.35) <= 0.011
        smoother_ok = (
            len(smoother_max) >= 3
            and abs(float(smoother_max[0]) - 0.35) <= 0.011
            and len(smoother_min) >= 3
            and abs(float(smoother_min[0]) - (-0.16)) <= 0.011
        )
        smoother_dynamics_ok = (
            len(smoother_accel) >= 3
            and abs(float(smoother_accel[0]) - 0.20) <= 0.011
            and abs(float(smoother_accel[2]) - 0.35) <= 0.011
            and len(smoother_decel) >= 3
            and abs(float(smoother_decel[0]) - (-0.28)) <= 0.011
            and abs(float(smoother_decel[2]) - (-0.40)) <= 0.011
        )
        cm_hardstop_ok = (
            len(cm_points) == len(CM_HARDSTOP_POINTS)
            and all(abs(float(a) - float(b)) <= 0.011 for a, b in zip(cm_points, CM_HARDSTOP_POINTS))
        )
        cm_footprint_topic_ok = CM_FOOTPRINT_TOPIC in cm_fp_topic_out
        cm_ttc_ok = abs(cm_ttc - CM_APPROACH_TTC_S) <= 0.011
        costmap_inf_clearing_ok = local_inf_valid and global_inf_valid
        dead_clearance_layer_removed = "clearance_layer" not in local_plugins_out

        try:
            gate_cmd = subprocess.check_output(
                ["pgrep", "-af", "turn_safety_gate_v3_FIX7.py"],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=3.0,
            ).strip()
        except Exception:
            gate_cmd = ""
        gate_v3_ok = (
            "--active" in gate_cmd
            and "--input-topic /cmd_vel" in gate_cmd
            and "--output-topic /cmd_vel_guarded" in gate_cmd
            and "--guard-angle-deg 25" in gate_cmd
            and "turn_safety_gate_v3_FIX7.py" in gate_cmd
        )

        # Verify the two Nav2 costmaps are listening for dynamic footprint updates.
        local_fp_subs = node._endpoint_fq_names(
            node.get_subscriptions_info_by_topic(LOCAL_FOOTPRINT_TOPIC)
        )
        global_fp_subs = node._endpoint_fq_names(
            node.get_subscriptions_info_by_topic(GLOBAL_FOOTPRINT_TOPIC)
        )
        footprint_topics_ok = (
            "/local_costmap/local_costmap" in local_fp_subs
            and "/global_costmap/global_costmap" in global_fp_subs
        )

        # Preflight publishes the already-baseline EMPTY footprint only; no motion.
        empty_profile_rec = node.set_load_profile("EMPTY", settle_s=0.20)

        chain_ok, chain_rec = node.command_chain_preflight()
        guarded_publishers = node._endpoint_fq_names(
            node.get_publishers_info_by_topic(CMD_GUARDED)
        )
        gate_ownership_ok = guarded_publishers == {
            "/amr_turn_safety_gate_v3_dynamic"
        }
        fresh_ok, fresh_rec = node.sensor_freshness_preflight()

        summary["preflight"] = {
            "required_nodes": "PASS",
            "controller_server_active": cs_active,
            "planner_server_active": ps_active,
            "bt_navigator_active": bt_active,
            "collision_monitor_active": cm_active,
            "velocity_smoother_active": vs_active,
            "controller_plugin": plugin_out,
            "mppi_ok": mppi_ok,
            "vx_max": vx_max,
            "vx_max_expected_0_35": vx_ok,
            "velocity_smoother_max_velocity": smoother_max,
            "velocity_smoother_min_velocity": smoother_min,
            "velocity_smoother_expected_plus0_35_minus0_16": smoother_ok,
            "velocity_smoother_max_accel": smoother_accel,
            "velocity_smoother_max_decel": smoother_decel,
            "velocity_smoother_expected_linear0_20_angular0_35_decel_minus0_28_minus0_40": smoother_dynamics_ok,
            "collision_monitor_hardstop_points": cm_points,
            "collision_monitor_hardstop_geometry_ok": cm_hardstop_ok,
            "collision_monitor_footprint_topic": cm_fp_topic_out,
            "collision_monitor_footprint_topic_ok": cm_footprint_topic_ok,
            "collision_monitor_approach_ttc_s": cm_ttc,
            "collision_monitor_approach_ttc_ok": cm_ttc_ok,
            "local_obstacle_inf_is_valid": local_inf_valid,
            "global_obstacle_inf_is_valid": global_inf_valid,
            "costmap_inf_clearing_ok": costmap_inf_clearing_ok,
            "local_costmap_plugins": local_plugins_out,
            "dead_clearance_layer_removed": dead_clearance_layer_removed,
            "turn_gate_process": gate_cmd,
            "turn_gate_v3_dynamic_guard_25_active": gate_v3_ok,
            "dynamic_footprint_topics_ok": footprint_topics_ok,
            "local_footprint_subscribers": sorted(local_fp_subs),
            "global_footprint_subscribers": sorted(global_fp_subs),
            "preflight_empty_profile": empty_profile_rec,
            "command_chain_ok": chain_ok,
            "command_chain": chain_rec,
            "cmd_vel_guarded_publishers": sorted(guarded_publishers),
            "gate_ownership_exclusive": gate_ownership_ok,
            "sensor_freshness_ok": fresh_ok,
            "sensor_freshness": fresh_rec,
        }

        if not all((cs_active, ps_active, bt_active, cm_active, vs_active)):
            print("FAIL PREFLIGHT: one or more lifecycle nodes are not ACTIVE")
            return finish(False, "PREFLIGHT_LIFECYCLE")
        if not mppi_ok:
            print(f"FAIL PREFLIGHT: FollowPath is not MPPI: {plugin_out}")
            return finish(False, "PREFLIGHT_NOT_MPPI")
        if not vx_ok:
            print(f"FAIL PREFLIGHT: MPPI vx_max={vx_max:.3f}, expected about 0.35 m/s")
            return finish(False, "PREFLIGHT_MPPI_SPEED")
        if not smoother_ok:
            print(
                f"FAIL PREFLIGHT: velocity_smoother max={smoother_max} min={smoother_min}, "
                "expected x about +0.35 / -0.16 m/s"
            )
            return finish(False, "PREFLIGHT_SMOOTHER_SPEED")
        if not smoother_dynamics_ok:
            print(
                f"FAIL PREFLIGHT: velocity_smoother accel={smoother_accel} decel={smoother_decel}, "
                "expected [0.20,0,0.35] / [-0.28,0,-0.40]"
            )
            return finish(False, "PREFLIGHT_SMOOTHER_DYNAMICS")
        if not costmap_inf_clearing_ok:
            print(
                "FAIL PREFLIGHT: FIX7 requires obstacle_layer.scan.inf_is_valid=true "
                "for BOTH local and global costmaps"
            )
            return finish(False, "PREFLIGHT_COSTMAP_INF_CLEARING")
        if not dead_clearance_layer_removed:
            print(
                "FAIL PREFLIGHT: dead clearance_layer is still loaded; "
                "/scan_empty_clearance had publisher_count=0 in verified runtime"
            )
            return finish(False, "PREFLIGHT_DEAD_CLEARANCE_LAYER")
        if not cm_hardstop_ok:
            print(
                f"FAIL PREFLIGHT: Collision Monitor BodyEmergencyStop.points={cm_points}, "
                f"expected {CM_HARDSTOP_POINTS}"
            )
            return finish(False, "PREFLIGHT_CM_HARDSTOP_GEOMETRY")
        if not cm_footprint_topic_ok:
            print(
                f"FAIL PREFLIGHT: Collision Monitor FootprintApproach topic mismatch: {cm_fp_topic_out}"
            )
            return finish(False, "PREFLIGHT_CM_FOOTPRINT_TOPIC")
        if not cm_ttc_ok:
            print(
                f"FAIL PREFLIGHT: Collision Monitor TTC={cm_ttc:.3f}s, "
                f"expected {CM_APPROACH_TTC_S:.2f}s"
            )
            return finish(False, "PREFLIGHT_CM_TTC")
        if not gate_v3_ok:
            print("FAIL PREFLIGHT: Turn Safety Gate V3 FIX7 is not the expected ACTIVE guard=25 setup")
            print(f"  process: {gate_cmd!r}")
            return finish(False, "PREFLIGHT_TURN_GATE_V3_25")
        if not footprint_topics_ok:
            print("FAIL PREFLIGHT: Nav2 costmaps are not both subscribed to dynamic footprint topics")
            print(f"  local subs : {sorted(local_fp_subs)}")
            print(f"  global subs: {sorted(global_fp_subs)}")
            return finish(False, "PREFLIGHT_DYNAMIC_FOOTPRINT")
        if not gate_ownership_ok:
            print("FAIL PREFLIGHT: /cmd_vel_guarded must have exactly one active publisher: V3 dynamic gate")
            print(f"  publishers: {sorted(guarded_publishers)}")
            return finish(False, "PREFLIGHT_GATE_OWNERSHIP")
        if not chain_ok:
            print("FAIL PREFLIGHT: safety command chain mismatch")
            for key, rec in chain_rec.items():
                if not rec["pass"]:
                    print(f"  MISSING {key} | seen={rec['seen']}")
            return finish(False, "PREFLIGHT_COMMAND_CHAIN")
        if not fresh_ok:
            print(f"FAIL PREFLIGHT: stale/missing odom or scan: {fresh_rec}")
            return finish(False, "PREFLIGHT_SENSORS")

        if not node.nav_client.wait_for_server(timeout_sec=4.0):
            return finish(False, "PREFLIGHT_NAV_ACTION")
        if not node.drive_client.wait_for_server(timeout_sec=4.0):
            return finish(False, "PREFLIGHT_DRIVE_ACTION")
        if not node.backup_client.wait_for_server(timeout_sec=4.0):
            return finish(False, "PREFLIGHT_BACKUP_ACTION")

        try:
            actuator_rec = actuator.verify_ready()
        except Exception as exc:
            print(f"FAIL PREFLIGHT ACTUATOR: {exc}")
            summary["preflight"]["actuator"] = {"result": "FAIL", "reason": str(exc)}
            return finish(False, "PREFLIGHT_ACTUATOR")
        summary["preflight"]["actuator"] = actuator_rec

        try:
            qr_cam_rec = qr_camera_preflight()
        except Exception as exc:
            print(f"FAIL PREFLIGHT QR CAMERA: {exc}")
            summary["preflight"]["qr_camera"] = {"result": "FAIL", "reason": str(exc)}
            return finish(False, "PREFLIGHT_QR_CAMERA")
        summary["preflight"]["qr_camera"] = qr_cam_rec

        print("PREFLIGHT PASS")
        print(f"  MPPI plugin : {plugin_out}")
        print(f"  MPPI vx_max : {vx_max:.3f} m/s")
        print(f"  smoother max: {smoother_max} | min: {smoother_min}")
        print(f"  smoother dyn: accel={smoother_accel} | decel={smoother_decel}")
        print(
            f"  CM hard-stop: {cm_points} | dynamic footprint topic OK | "
            f"TTC={cm_ttc:.2f}s"
        )
        print("  turn gate   : V3 FIX7 DYNAMIC ACTIVE guard=25 deg")
        print(f"  sensor age  : odom={fresh_rec['odom_age_s']:.3f}s scan={fresh_rec['scan_age_s']:.3f}s")
        print("  footprint   : EMPTY dynamic publishers connected to both costmaps")
        print("  safety chain: PASS")
        print(
            f"  actuator    : {ACTUATOR_EXPECTED_VERSION} @ "
            f"{actuator_rec.get('port')} -> {actuator_rec.get('resolved_port')} PASS"
        )
        print(f"  QR camera   : {QR_CAMERA_BY_ID} | expected={QR_EXPECTED_B2!r} PASS")

        if args.preflight_only:
            summary["success"] = True
            summary["result"] = "PREFLIGHT_PASS_ONLY"
            summary["failed_stage"] = None
            summary_path = output_dir / "mission_summary.json"
            summary_path.write_text(
                json.dumps(json_safe(summary), indent=2, ensure_ascii=False)
            )
            finish_called = True
            actuator.close()
            print()
            print("PREFLIGHT-ONLY PASS: NO MOTION GOAL WAS SENT")
            print(f"SUMMARY: {summary_path}")
            return 0

        if args.qr_test_only:
            qr_rec = scan_b2_qr_gate()
            summary["stages"].append({"stage": "B2_QR_GATE_TEST_ONLY", **qr_rec})
            summary["success"] = True
            summary["failed_stage"] = None
            summary["result"] = f"QR_TEST_ONLY_{qr_rec['result']}"
            summary_path = output_dir / "mission_summary.json"
            summary_path.write_text(json.dumps(json_safe(summary), indent=2, ensure_ascii=False))
            finish_called = True
            actuator.close()
            print(f"QR TEST ONLY RESULT: {qr_rec['result']} data={qr_rec.get('data')!r}")
            print("NO MOTION GOAL WAS SENT")
            print(f"SUMMARY: {summary_path}")
            return 0

        # Start bag only after no-motion preflight passes.
        if not args.no_bag:
            bag_proc, bag_log, bag_topics = start_bag_recording(output_dir)
            summary["bag_topics"] = bag_topics
            node.spin_for(1.0)

        # Ensure runtime starts in EMPTY geometry.
        empty_rec = node.set_load_profile("EMPTY")
        summary["stages"].append({"stage": "FOOTPRINT_EMPTY_START", **empty_rec})

        # ------------------------------------------------------------------
        # START CHECK AT A_H
        # ------------------------------------------------------------------
        start_pose = node.stable_map_pose()
        if start_pose is None:
            return finish(False, "START_POSE")

        start_xy, start_yaw_err = pose_error(start_pose, stations["A_H"])
        summary["start_pose"] = {
            "x": start_pose[0],
            "y": start_pose[1],
            "yaw_deg": math.degrees(start_pose[2]),
            "A_H_xy_error_cm": start_xy * 100.0,
            "A_H_yaw_error_deg": math.degrees(start_yaw_err),
        }

        print()
        print(
            f"START MAP: x={start_pose[0]:+.3f} y={start_pose[1]:+.3f} "
            f"yaw={math.degrees(start_pose[2]):+.3f} deg"
        )
        print(
            f"A_H ERROR: XY={start_xy*100.0:.2f} cm | "
            f"YAW={math.degrees(start_yaw_err):+.3f} deg"
        )

        if not args.skip_home_check and start_xy > START_HOME_MAX_XY_M:
            print(
                f"REFUSE START: robot is {start_xy:.3f} m from A_H, "
                f"limit={START_HOME_MAX_XY_M:.2f} m"
            )
            return finish(False, "START_NOT_AT_A_H")

        # ------------------------------------------------------------------
        # 1) NAV2 -> B1/B2 (coarse XY) + one-shot yaw
        # ------------------------------------------------------------------
        ok, rec = node.navigate_with_escape_recovery(route, stations[route], bt_path)
        summary["stages"].append({"stage": f"NAV_{route}", **rec})
        if not ok:
            return finish(False, f"NAV_{route}")

        ok, rec = node.one_shot_yaw_align(route, stations[route])
        summary["stages"].append({"stage": f"ALIGN_{route}", **rec})
        if not ok:
            return finish(False, f"ALIGN_{route}")

        # ------------------------------------------------------------------
        # 2) STOP -> QR AUTH GATE -> only valid BOX_001 is allowed to PICK
        # Camera does not control yaw, steering, or robot position.
        # ------------------------------------------------------------------
        if not node.wait_robot_stopped(timeout_s=3.0):
            return finish(False, f"{route}_NOT_STOPPED_BEFORE_QR")

        try:
            qr_rec = scan_b2_qr_gate()
        except Exception as exc:
            print(f"QR GATE FAIL: {exc}")
            return finish(False, f"{route}_QR_CAMERA")
        summary["stages"].append({"stage": f"{route}_QR_GATE", **qr_rec})

        if qr_rec["result"] != "QR_VALID":
            print(f"QR PICK DENIED: result={qr_rec['result']} data={qr_rec.get('data')!r}")
            print("RETURNING TO A_H WITHOUT OPEN/PICK")
            ok, rec = node.navigate_with_escape_recovery("A_H", stations["A_H"], bt_path)
            summary["stages"].append({"stage": "QR_REJECT_RETURN_A_H", **rec})
            if not ok:
                return finish(False, "QR_REJECT_RETURN_A_H")
            ok, rec = node.one_shot_yaw_align("A_H", stations["A_H"])
            summary["stages"].append({"stage": "QR_REJECT_ALIGN_A_H", **rec})
            if not ok:
                return finish(False, "QR_REJECT_ALIGN_A_H")
            return finish_qr_reject(qr_rec["result"])

        # ------------------------------------------------------------------
        # 3) QR VALID -> OPEN FULL -> PICKUP STRAIGHT IN -> PICK -> PICK_DONE
        # ------------------------------------------------------------------
        if not node.wait_robot_stopped(timeout_s=3.0):
            return finish(False, f"{route}_NOT_STOPPED_BEFORE_OPEN")

        try:
            rec = actuator.open_full()
        except Exception as exc:
            print(f"ACTUATOR OPEN FAIL: {exc}")
            return finish(False, f"{route}_ACTUATOR_OPEN")
        summary["stages"].append({"stage": f"{route}_OPEN_FULL", **rec})

        label_fwd = f"{route}_PICK_FORWARD"
        ok, rec = node.drive_on_heading_persistent(
            pickup_distance_m,
            float(args.pickup_speed),
            label_fwd,
        )
        summary["stages"].append({"stage": label_fwd, **rec})
        if not ok:
            return finish(False, label_fwd)

        if not node.wait_robot_stopped(timeout_s=3.0):
            return finish(False, f"{route}_NOT_STOPPED_BEFORE_PICK")

        try:
            rec = actuator.pick()
        except Exception as exc:
            print(f"ACTUATOR PICK FAIL: {exc}")
            return finish(False, f"{route}_ACTUATOR_PICK")
        summary["stages"].append({"stage": f"{route}_PICK_DONE", **rec})

        # TEST-ONLY: keep the same physical footprint after PICK.
        # We still publish LOADED state for mission bookkeeping/gate state, but
        # FIX7 LOADED geometry is intentionally identical to EMPTY geometry.
        loaded_rec = node.set_load_profile("LOADED")
        summary["stages"].append({"stage": "FOOTPRINT_LOADED_AFTER_PICK", **loaded_rec})

        # ------------------------------------------------------------------
        # 3) EXPLICIT MISSION BACKUP: exit the pickup zone before Nav2 turns
        # ------------------------------------------------------------------
        label_back = f"{route}_PICK_EXIT_BACKUP"
        ok, rec = node.backup_persistent(
            exit_backup_distance_m,
            float(args.backup_speed),
            label_back,
        )
        summary["stages"].append({"stage": label_back, **rec})
        if not ok:
            return finish(False, label_back)

        # ------------------------------------------------------------------
        # 4) NAV2 -> TH + yaw lock
        # ------------------------------------------------------------------
        ok, rec = node.navigate_with_escape_recovery("TH", stations["TH"], bt_path)
        summary["stages"].append({"stage": "NAV_TH", **rec})
        if not ok:
            return finish(False, "NAV_TH")

        ok, rec = node.one_shot_yaw_align("TH", stations["TH"])
        summary["stages"].append({"stage": "ALIGN_TH", **rec})
        if not ok:
            return finish(False, "ALIGN_TH")

        # ------------------------------------------------------------------
        # 5) TH DROP: +0.40 m ODOM YAW HOLD -> DROP -> -0.40 m -> CLOSE empty
        # ------------------------------------------------------------------
        ok, rec = node.drive_straight_heading_lock(
            TH_DROP_FORWARD_M,
            TH_DROP_FORWARD_SPEED_MPS,
            "TH_DROP_FORWARD_40CM",
        )
        summary["stages"].append({"stage": "TH_DROP_FORWARD_40CM", **rec})
        if not ok:
            return finish(False, "TH_DROP_FORWARD_40CM")

        if not node.wait_robot_stopped(timeout_s=3.0):
            return finish(False, "TH_NOT_STOPPED_BEFORE_DROP")

        try:
            rec = actuator.drop()
        except Exception as exc:
            print(f"ACTUATOR DROP FAIL: {exc}")
            return finish(False, "TH_ACTUATOR_DROP")
        summary["stages"].append({"stage": "TH_DROP_DONE", **rec})

        ok, rec = node.backup_persistent(
            TH_DROP_BACKUP_M,
            TH_DROP_BACKUP_SPEED_MPS,
            "TH_DROP_BACKUP_40CM",
        )
        summary["stages"].append({"stage": "TH_DROP_BACKUP_40CM", **rec})
        if not ok:
            return finish(False, "TH_DROP_BACKUP_40CM")

        if not node.wait_robot_stopped(timeout_s=3.0):
            return finish(False, "TH_NOT_STOPPED_BEFORE_CLOSE")

        try:
            rec = actuator.close_empty_after_drop()
        except Exception as exc:
            print(f"ACTUATOR CLOSE FAIL: {exc}")
            return finish(False, "TH_ACTUATOR_CLOSE")
        summary["stages"].append({"stage": "TH_CLOSE_DONE", **rec})

        # Mission bookkeeping returns to EMPTY after DROP retreat. In FIX7 TEST,
        # LOADED and EMPTY geometry are identical, so no physical footprint size
        # change occurs at either transition.
        empty_rec = node.set_load_profile("EMPTY")
        summary["stages"].append({"stage": "FOOTPRINT_EMPTY_AFTER_DROP_RETREAT_AND_CLOSE", **empty_rec})

        # ------------------------------------------------------------------
        # 6) NAV2 -> A_H + final yaw lock
        # ------------------------------------------------------------------
        ok, rec = node.navigate_with_escape_recovery("A_H", stations["A_H"], bt_path)
        summary["stages"].append({"stage": "NAV_A_H", **rec})
        if not ok:
            return finish(False, "NAV_A_H")

        ok, rec = node.one_shot_yaw_align("A_H", stations["A_H"])
        summary["stages"].append({"stage": "ALIGN_A_H", **rec})
        if not ok:
            return finish(False, "ALIGN_A_H")

        return finish(True)

    except KeyboardInterrupt:
        print("\nCTRL-C: mission interrupted")
        return finish(False, "USER_ABORT")

    except Exception as exc:
        print(f"\nUNHANDLED ERROR: {type(exc).__name__}: {exc}")
        summary["exception"] = f"{type(exc).__name__}: {exc}"
        return finish(False, "EXCEPTION")

    finally:
        if not finish_called and rclpy.ok():
            try:
                node.safe_stop_all("FINALIZER_LAST_CHANCE", wait_stop_s=3.0)
            except Exception as exc:
                print(f"FINALIZER SAFE STOP ERROR: {type(exc).__name__}: {exc}")

        try:
            if not finish_called:
                actuator.safe_stop()
            actuator.close()
            node.destroy_node()
        except Exception:
            pass

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
