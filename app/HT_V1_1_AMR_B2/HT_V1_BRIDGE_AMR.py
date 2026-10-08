#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""ROS 2 -> localhost bridge cho HT_V1 AMR B2.

READ-ONLY by design:
- Subscribes to ROS topics / TF.
- Polls Supervisor STATUS over loopback UDP only.
- Reads hardware presence from /dev paths.
- Sends JSON telemetry to 127.0.0.1:8767.
- Publishes no ROS topics and sends no motion/actuator/mission commands.
"""
from __future__ import annotations

import json
import math
import os
import socket
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.duration import Duration

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path as NavPath
from sensor_msgs.msg import LaserScan
import tf2_ros

from HT_V1_MON_AMR import collect_runtime_monitor

UDP_GUI_ADDR = ("127.0.0.1", 8767)
SUPERVISOR_ADDR = ("127.0.0.1", 5005)
SEND_HZ = 5.0

DRIVE_BY_ID = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_376B36513235-if00"
ACT_BY_ID = "/dev/serial/by-id/usb-STMicroelectronics_STM32_Virtual_ComPort_379336853235-if00"
LIDAR_BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
CAM_BY_ID = "/dev/v4l/by-id/usb-XIFT_Web_Camera_SN00010-video-index0"
STATIONS_FILE = Path.home() / "amr_ws/amr_new_nav/mission_v1_clean/stations_v1_clean.yaml"


def q_yaw(q) -> float:
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def age(now: float, stamp: float) -> Optional[float]:
    return None if stamp <= 0 else max(0.0, now - stamp)


def status_from_age(stamp: float, limit: float = 1.5) -> bool:
    return stamp > 0 and (time.monotonic() - stamp) < limit


def transform_xy(tx: float, ty: float, yaw: float, x: float, y: float) -> Tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return tx + c * x - s * y, ty + s * x + c * y


def grid_world(meta, ix: int, iy: int) -> Tuple[float, float]:
    ox = meta.origin.position.x
    oy = meta.origin.position.y
    oyaw = q_yaw(meta.origin.orientation)
    lx = (ix + 0.5) * meta.resolution
    ly = (iy + 0.5) * meta.resolution
    return transform_xy(ox, oy, oyaw, lx, ly)


class AMRGuiBridge(Node):
    def __init__(self) -> None:
        super().__init__("ht_v1_amr_gui_monitor")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sup_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sup_sock.settimeout(0.08)

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=8.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self, spin_thread=False)

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        reliable = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)

        self.latest_scan: Optional[LaserScan] = None
        self.latest_odom: Optional[Odometry] = None
        self.latest_amcl: Optional[PoseWithCovarianceStamped] = None
        self.latest_map: Optional[OccupancyGrid] = None
        self.latest_global_costmap: Optional[OccupancyGrid] = None
        self.latest_local_costmap: Optional[OccupancyGrid] = None

        self.scan_t = self.odom_t = self.amcl_t = self.map_t = 0.0
        self.global_costmap_t = self.local_costmap_t = 0.0
        self.global_costmap_points_cache: List[dict] = []
        self.local_costmap_points_cache: List[dict] = []
        self.map_revision = 0
        self.last_map_sent_t = 0.0
        self.map_dirty = False
        self.seq = 0

        self.path_subs: Dict[str, object] = {}
        self.path_data: Dict[str, Tuple[float, List[dict]]] = {}
        self.last_discovery = 0.0
        self.supervisor = {}
        self.supervisor_t = 0.0
        self.route_markers = self._load_route_markers()

        self.create_subscription(OccupancyGrid, "/map", self.cb_map, latched)
        self.create_subscription(LaserScan, "/scan", self.cb_scan, qos_profile_sensor_data)
        self.create_subscription(Odometry, "/odom", self.cb_odom, qos_profile_sensor_data)
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.cb_amcl, reliable)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self.cb_global_costmap, qos_profile_sensor_data)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self.cb_local_costmap, qos_profile_sensor_data)

        self.create_timer(2.0, self.discover_paths)
        self.create_timer(1.0, self.poll_supervisor)
        self.create_timer(1.0 / SEND_HZ, self.send_packet)
        self.get_logger().info("V1.4 monitor bridge started; no publishers / no cmd_vel / camera device not opened")

    def _load_route_markers(self) -> List[dict]:
        # Optional. GUI still works without this file / PyYAML.
        try:
            import yaml
            data = yaml.safe_load(STATIONS_FILE.read_text(encoding="utf-8"))
        except Exception:
            return []
        wanted = {"A_H", "B2", "TH"}
        found: Dict[str, dict] = {}

        def walk(obj, inherited_name=None):
            if isinstance(obj, dict):
                name = obj.get("name", inherited_name)
                if name in wanted and "x" in obj and "y" in obj:
                    try:
                        found[name] = {"label": name, "x": float(obj["x"]), "y": float(obj["y"])}
                    except Exception:
                        pass
                for k, v in obj.items():
                    kname = k if isinstance(k, str) and k in wanted else name
                    if isinstance(v, dict) and k in wanted and "x" in v and "y" in v:
                        try:
                            found[k] = {"label": k, "x": float(v["x"]), "y": float(v["y"])}
                        except Exception:
                            pass
                    walk(v, kname)
            elif isinstance(obj, list):
                for v in obj:
                    walk(v, inherited_name)

        walk(data)
        return [found[k] for k in ("A_H", "B2", "TH") if k in found]

    def cb_map(self, msg: OccupancyGrid) -> None:
        self.latest_map = msg
        self.map_t = time.monotonic()
        self.map_revision += 1
        self.map_dirty = True

    def cb_scan(self, msg: LaserScan) -> None:
        self.latest_scan = msg
        self.scan_t = time.monotonic()

    def cb_odom(self, msg: Odometry) -> None:
        self.latest_odom = msg
        self.odom_t = time.monotonic()

    def cb_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.latest_amcl = msg
        self.amcl_t = time.monotonic()

    def cb_global_costmap(self, msg: OccupancyGrid) -> None:
        self.latest_global_costmap = msg
        self.global_costmap_t = time.monotonic()
        self.global_costmap_points_cache = self.costmap_points(msg, self.global_costmap_t, 700)

    def cb_local_costmap(self, msg: OccupancyGrid) -> None:
        self.latest_local_costmap = msg
        self.local_costmap_t = time.monotonic()
        self.local_costmap_points_cache = self.costmap_points(msg, self.local_costmap_t, 850)

    def discover_paths(self) -> None:
        try:
            topics = self.get_topic_names_and_types()
        except Exception:
            return
        for topic, types in topics:
            if topic in self.path_subs:
                continue
            if "nav_msgs/msg/Path" not in types:
                continue
            low = topic.lower()
            if not any(k in low for k in ("plan", "path")):
                continue
            def cb(msg, t=topic):
                pts = [{"x": round(p.pose.position.x, 4), "y": round(p.pose.position.y, 4)} for p in msg.poses]
                if len(pts) > 1200:
                    step = max(1, len(pts) // 1200)
                    pts = pts[::step]
                self.path_data[t] = (time.monotonic(), pts)
            try:
                self.path_subs[topic] = self.create_subscription(NavPath, topic, cb, QoSProfile(depth=5))
                self.get_logger().info(f"Monitoring Path topic: {topic}")
            except Exception:
                pass

    def poll_supervisor(self) -> None:
        try:
            self.sup_sock.sendto(b"STATUS", SUPERVISOR_ADDR)
            raw, _ = self.sup_sock.recvfrom(4096)
            txt = raw.decode(errors="replace").strip()
            if not txt.startswith("STATUS "):
                return
            out = {}
            for token in txt[len("STATUS "):].split():
                if "=" in token:
                    k, v = token.split("=", 1)
                    out[k] = v
            out["raw"] = txt
            self.supervisor = out
            self.supervisor_t = time.monotonic()
        except Exception:
            self.supervisor = {}

    def current_pose(self) -> Optional[dict]:
        try:
            tr = self.tf_buffer.lookup_transform("map", "base_link", Time(), timeout=Duration(seconds=0.03))
            yaw = q_yaw(tr.transform.rotation)
            return {
                "x": round(tr.transform.translation.x, 4),
                "y": round(tr.transform.translation.y, 4),
                "yaw": round(math.degrees(yaw), 2),
                "source": "TF map->base_link",
            }
        except Exception:
            pass
        if self.latest_amcl is not None and status_from_age(self.amcl_t, 2.0):
            p = self.latest_amcl.pose.pose
            return {
                "x": round(p.position.x, 4),
                "y": round(p.position.y, 4),
                "yaw": round(math.degrees(q_yaw(p.orientation)), 2),
                "source": "/amcl_pose",
            }
        return None

    def current_velocity(self) -> dict:
        if self.latest_odom is None or not status_from_age(self.odom_t, 1.5):
            return {"linear": 0.0, "angular": 0.0}
        t = self.latest_odom.twist.twist
        return {"linear": round(t.linear.x, 4), "angular": round(t.angular.z, 4)}

    def scan_points(self) -> List[dict]:
        scan = self.latest_scan
        if scan is None or not status_from_age(self.scan_t, 1.2):
            return []
        frame = scan.header.frame_id or "laser"
        try:
            tr = self.tf_buffer.lookup_transform("map", frame, Time(), timeout=Duration(seconds=0.03))
            tx, ty = tr.transform.translation.x, tr.transform.translation.y
            yaw = q_yaw(tr.transform.rotation)
        except Exception:
            return []
        n = len(scan.ranges)
        step = max(1, n // 300)
        pts = []
        for i in range(0, n, step):
            r = scan.ranges[i]
            if not math.isfinite(r) or r < max(0.05, scan.range_min) or r > scan.range_max:
                continue
            a = scan.angle_min + i * scan.angle_increment
            lx, ly = r * math.cos(a), r * math.sin(a)
            x, y = transform_xy(tx, ty, yaw, lx, ly)
            pts.append({"x": round(x, 3), "y": round(y, 3)})
        return pts[:360]

    def costmap_points(self, msg: Optional[OccupancyGrid], fresh_t: float, max_points: int) -> List[dict]:
        if msg is None or not status_from_age(fresh_t, 2.0):
            return []
        idxs = [i for i, v in enumerate(msg.data) if v >= 65]
        if not idxs:
            return []
        step = max(1, len(idxs) // max_points)
        pts = []
        w = msg.info.width
        for flat in idxs[::step][:max_points]:
            ix, iy = flat % w, flat // w
            x, y = grid_world(msg.info, ix, iy)
            pts.append({"x": round(x, 3), "y": round(y, 3)})
        return pts

    def paths(self) -> Tuple[List[dict], List[dict], dict]:
        now = time.monotonic()
        global_candidates = []
        local_candidates = []
        ages = {}
        for topic, (stamp, pts) in self.path_data.items():
            a = now - stamp
            ages[topic] = round(a, 2)
            if a > 8.0 or not pts:
                continue
            if "local" in topic.lower():
                local_candidates.append((stamp, topic, pts))
            else:
                global_candidates.append((stamp, topic, pts))
        global_path = max(global_candidates, default=(0, "", []), key=lambda x: x[0])[2]
        local_path = max(local_candidates, default=(0, "", []), key=lambda x: x[0])[2]
        return global_path, local_path, ages

    def encode_map(self) -> Optional[dict]:
        msg = self.latest_map
        if msg is None:
            return None
        data = list(msg.data)
        stride = 1
        while True:
            w0, h0 = msg.info.width, msg.info.height
            w = (w0 + stride - 1) // stride
            h = (h0 + stride - 1) // stride
            sampled = []
            for iy in range(0, h0, stride):
                row = iy * w0
                for ix in range(0, w0, stride):
                    sampled.append(int(data[row + ix]))
            rle = []
            if sampled:
                prev, count = sampled[0], 1
                for v in sampled[1:]:
                    if v == prev and count < 65535:
                        count += 1
                    else:
                        rle.append([prev, count])
                        prev, count = v, 1
                rle.append([prev, count])
            out = {
                "revision": self.map_revision,
                "frame": msg.header.frame_id or "map",
                "width": w,
                "height": h,
                "resolution": round(msg.info.resolution * stride, 6),
                "origin_x": round(msg.info.origin.position.x, 6),
                "origin_y": round(msg.info.origin.position.y, 6),
                "origin_yaw": round(q_yaw(msg.info.origin.orientation), 8),
                "stride": stride,
                "rle": rle,
            }
            # Leave room for other telemetry in a UDP datagram.
            if len(json.dumps(out, separators=(",", ":")).encode()) < 38000 or stride >= 8:
                return out
            stride *= 2

    def node_status(self) -> Tuple[dict, List[str]]:
        try:
            names = set(self.get_node_names())
        except Exception:
            names = set()
        def has(name: str) -> bool:
            return name in names or f"/{name}" in names
        lidar = has("rplidar_node") and status_from_age(self.scan_t, 1.5)
        drive = has("amr_stm32_bridge") and status_from_age(self.odom_t, 1.5)
        nav2 = has("amcl") and has("controller_server") and has("planner_server")
        collision = has("collision_monitor")
        return {
            "lidar": "ONLINE" if lidar else ("PRESENT" if os.path.exists(LIDAR_BY_ID) else "OFFLINE"),
            "camera": "PRESENT" if os.path.exists(CAM_BY_ID) else "OFFLINE",
            "stm32_drive": "ONLINE" if drive else ("PRESENT" if os.path.exists(DRIVE_BY_ID) else "OFFLINE"),
            "stm32_actuator": "PRESENT" if os.path.exists(ACT_BY_ID) else "OFFLINE",
            "nav2": "ONLINE" if nav2 else "OFFLINE",
            "collision_monitor": "ONLINE" if collision else "OFFLINE",
        }, sorted(names)

    def send_packet(self) -> None:
        now = time.monotonic()
        pose = self.current_pose()
        velocity = self.current_velocity()
        global_path, local_path, path_ages = self.paths()
        hw, node_names = self.node_status()
        sup_live = status_from_age(self.supervisor_t, 2.0) and bool(self.supervisor)
        sup = dict(self.supervisor) if sup_live else {}

        ros_live = any([
            status_from_age(self.map_t, 3.0),
            status_from_age(self.scan_t, 1.5),
            status_from_age(self.amcl_t, 2.0),
            pose is not None,
        ])
        state = sup.get("state", "ROS LIVE" if ros_live else "WAITING ROS")
        mission_pid = sup.get("mission_pid", "-")
        monitor = collect_runtime_monitor(sup if sup_live else {})
        mission = monitor.get("stage", "B2 RUNNING" if mission_pid not in ("-", "", None) else ("IDLE" if sup_live else "MONITOR"))
        esp32 = "ONLINE" if sup_live and sup.get("esp32", "-") != "-" else "OFFLINE"

        pkt = {
            "seq": self.seq,
            "source": "ROS2_REALTIME_RECOVERY_MONITOR",
            "ros_live": ros_live,
            "system": "ROS LIVE" if ros_live else "WAITING ROS",
            "robot_state": state,
            "mission": mission,
            "esp32": esp32,
            "sensor1": sup.get("sensor1", "UNKNOWN"),
            "sensor2": sup.get("sensor2", "UNKNOWN"),
            "pose": pose,
            "velocity": velocity,
            "route_markers": self.route_markers,
            "global_path": global_path,
            "local_path": local_path,
            "scan_points": self.scan_points(),
            "costmap_obstacles": self.local_costmap_points_cache if status_from_age(self.local_costmap_t, 2.0) else [],
            "global_costmap_obstacles": self.global_costmap_points_cache if status_from_age(self.global_costmap_t, 2.0) else [],
            "hardware": hw,
            "supervisor": sup,
            "mission_monitor": monitor,
            "qr": monitor.get("qr", {}),
            "event_log": monitor.get("events", []),
            "log_revision": monitor.get("log_revision", "0"),
            "topics": {
                "map_age_s": age(now, self.map_t),
                "scan_age_s": age(now, self.scan_t),
                "odom_age_s": age(now, self.odom_t),
                "amcl_age_s": age(now, self.amcl_t),
                "path_ages": path_ages,
            },
            "timestamp": time.time(),
        }
        if self.map_dirty or (self.latest_map is not None and (now - self.last_map_sent_t) > 5.0):
            m = self.encode_map()
            if m is not None:
                pkt["map"] = m
                self.map_dirty = False
                self.last_map_sent_t = now

        try:
            raw = json.dumps(pkt, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            # UDP hard limit. If unexpectedly oversized, preserve the critical data.
            if len(raw) > 64000:
                pkt["scan_points"] = pkt["scan_points"][::2]
                pkt["costmap_obstacles"] = pkt["costmap_obstacles"][::2]
                pkt["global_costmap_obstacles"] = []
                raw = json.dumps(pkt, ensure_ascii=False, separators=(",", ":")).encode("utf-8")
            if len(raw) <= 65000:
                self.sock.sendto(raw, UDP_GUI_ADDR)
        except Exception:
            pass
        self.seq += 1


def main() -> None:
    rclpy.init()
    node = AMRGuiBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
