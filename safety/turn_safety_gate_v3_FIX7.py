#!/usr/bin/env python3
"""
AMR TURN SAFETY GATE V3 FIX7 DYNAMIC
=======================

TEST module for ROS 2 Humble.

Goal
----
Protect near-in-place turning commands using the REAL LiDAR /scan and the
EMPTY/LOADED mission state with the SAME test footprint geometry.

Default mode is SHADOW:
- subscribes /cmd_vel_nav and /scan
- evaluates whether LEFT/RIGHT pivot is safe
- logs ALLOW/BLOCK decisions
- sends NO motion command

Active mode (--active):
- subscribes /cmd_vel_nav
- publishes guarded commands to /cmd_vel_nav_guarded
- pure/normal linear commands pass unchanged
- unsafe near-pivot commands are replaced by ZERO Twist
- stale/no scan or TF blocks only near-pivot commands (fail-closed for turn)
- it does NOT replace Collision Monitor; downstream safety must remain active

Important integration rule
--------------------------
Do NOT run --active in the existing command chain until the input/output topics
are deliberately rewired so that:
    MPPI/controller -> /cmd_vel_nav -> Velocity Smoother -> /cmd_vel
    -> Turn Safety Gate -> /cmd_vel_guarded
    -> Collision Monitor -> /cmd_vel_safe -> bridge -> STM32

Never publish guarded output back onto /cmd_vel_nav, otherwise a feedback loop
would be created.

Geometry
--------
Physical FIX7 TEST footprint (EMPTY = LOADED):
    front +0.943 m
    rear  -0.455 m
    left  +0.370 m
    right -0.370 m

Default Nav2 footprint padding:
    0.05 m

V1 protects near-pivot commands only. Moving arcs are passed unchanged and will
be handled in a later V2 trajectory gate.

Examples
--------
SAFE shadow test:
  python3 turn_safety_gate_v1.py --shadow

After deliberate topic rewiring:
  python3 turn_safety_gate_v1.py --active
"""

import argparse
import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


def yaw_from_quat(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def inside_rect(x, y, front, rear, left, right):
    return (-rear <= x <= front) and (-right <= y <= left)


def rotate_obstacle_into_future_robot_frame(x, y, robot_yaw):
    """
    Robot rotates by robot_yaw about base_link while obstacle stays fixed.
    Express current obstacle point in future robot frame:
        p_future = R(-robot_yaw) * p_now
    """
    c = math.cos(robot_yaw)
    s = math.sin(robot_yaw)
    xr = c * x + s * y
    yr = -s * x + c * y
    return xr, yr


@dataclass
class DirectionRisk:
    safe: bool
    first_contact_deg: Optional[float]
    hit_count: int
    hit_xy: Optional[Tuple[float, float]]
    reason: str


class TurnSafetyGate(Node):
    def __init__(self, args):
        super().__init__("amr_turn_safety_gate_v3_dynamic")
        self.args = args

        self.profile = "EMPTY"
        self._apply_profile("EMPTY")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.scan_points = []
        self.last_scan_mono = 0.0
        self.last_scan_frame = ""
        self.scan_tf_ok = False

        self.left_risk = DirectionRisk(
            safe=False, first_contact_deg=None, hit_count=0,
            hit_xy=None, reason="NO_SCAN"
        )
        self.right_risk = DirectionRisk(
            safe=False, first_contact_deg=None, hit_count=0,
            hit_xy=None, reason="NO_SCAN"
        )

        self.cmd_count = 0
        self.block_count = 0
        self.last_log_mono = 0.0
        self.last_decision = ""

        self.create_subscription(
            LaserScan,
            args.scan_topic,
            self.scan_cb,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Twist,
            args.input_topic,
            self.cmd_cb,
            20,
        )
        load_state_qos = QoSProfile(depth=1)
        load_state_qos.reliability = ReliabilityPolicy.RELIABLE
        load_state_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            String,
            args.load_state_topic,
            self.load_state_cb,
            load_state_qos,
        )

        self.status_pub = self.create_publisher(
            String,
            "/amr/turn_safety_gate/status",
            10,
        )

        self.out_pub = None
        if args.active:
            self.out_pub = self.create_publisher(
                Twist,
                args.output_topic,
                20,
            )

        mode = "ACTIVE" if args.active else "SHADOW"
        self.get_logger().info(
            f"TURN SAFETY GATE V3 DYNAMIC | mode={mode} | "
            f"in={args.input_topic} | out={args.output_topic} | "
            f"guard={args.guard_angle_deg:.1f}deg | "
            f"pivot_v<={args.pivot_linear_max:.3f}m/s | "
            f"pivot_w>={args.pivot_angular_min:.3f}rad/s | "
            f"profile={self.profile} | state_topic={args.load_state_topic}"
        )

    def _apply_profile(self, profile):
        profile = str(profile).upper()
        if profile == "LOADED":
            front = self.args.loaded_front
            rear = self.args.loaded_rear
            left = self.args.loaded_left
            right = self.args.loaded_right
        else:
            profile = "EMPTY"
            front = self.args.front
            rear = self.args.rear
            left = self.args.left
            right = self.args.right

        self.profile = profile
        self.front = front + self.args.padding
        self.rear = rear + self.args.padding
        self.left = left + self.args.padding
        self.right = right + self.args.padding

    def load_state_cb(self, msg):
        requested = str(msg.data).strip().upper()
        if requested not in ("EMPTY", "LOADED"):
            self.get_logger().warning(f"IGNORE unknown load state: {msg.data!r}")
            return
        if requested != self.profile:
            self._apply_profile(requested)
            self.get_logger().info(
                f"PROFILE -> {self.profile} | padded "
                f"front=+{self.front:.3f} rear=-{self.rear:.3f} "
                f"left=+{self.left:.3f} right=-{self.right:.3f}"
            )

    # ------------------------------------------------------------------
    # Scan processing
    # ------------------------------------------------------------------
    def scan_cb(self, msg):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.args.base_frame,
                msg.header.frame_id,
                Time(),
            )
        except TransformException as exc:
            self.scan_tf_ok = False
            self.scan_points = []
            self.left_risk = DirectionRisk(
                False, None, 0, None, f"TF_FAIL:{exc}"
            )
            self.right_risk = DirectionRisk(
                False, None, 0, None, f"TF_FAIL:{exc}"
            )
            return

        tr = tf.transform.translation
        yaw = yaw_from_quat(tf.transform.rotation)
        cy = math.cos(yaw)
        sy = math.sin(yaw)

        rmin = max(float(msg.range_min), self.args.min_range)
        rmax = min(float(msg.range_max), self.args.max_range)

        pts = []
        angle = float(msg.angle_min)

        for rv in msg.ranges:
            r = float(rv)
            if math.isfinite(r) and rmin <= r <= rmax:
                xl = r * math.cos(angle)
                yl = r * math.sin(angle)

                xb = float(tr.x) + cy * xl - sy * yl
                yb = float(tr.y) + sy * xl + cy * yl

                # Ignore any unexpected self-hit points already in footprint.
                # Their count is still reflected by diagnostics via reason.
                pts.append((xb, yb))

            angle += msg.angle_increment

        self.scan_points = pts
        self.scan_tf_ok = True
        self.last_scan_mono = time.monotonic()
        self.last_scan_frame = msg.header.frame_id

        inside_now = sum(
            1 for x, y in pts
            if inside_rect(
                x, y,
                self.front, self.rear,
                self.left, self.right,
            )
        )

        if inside_now > 0:
            reason = f"SELF_HIT:{inside_now}"
            self.left_risk = DirectionRisk(False, 0.0, inside_now, None, reason)
            self.right_risk = DirectionRisk(False, 0.0, inside_now, None, reason)
            return

        self.left_risk = self.evaluate_direction(+1)
        self.right_risk = self.evaluate_direction(-1)

    def evaluate_direction(self, sign):
        if not self.scan_points:
            return DirectionRisk(False, None, 0, None, "NO_POINTS")

        steps = int(
            math.ceil(self.args.guard_angle_deg / self.args.step_deg)
        )

        for k in range(1, steps + 1):
            angle_abs = min(
                k * self.args.step_deg,
                self.args.guard_angle_deg,
            )
            theta = math.radians(sign * angle_abs)

            hits = []
            for x, y in self.scan_points:
                xr, yr = rotate_obstacle_into_future_robot_frame(
                    x, y, theta
                )
                if inside_rect(
                    xr, yr,
                    self.front, self.rear,
                    self.left, self.right,
                ):
                    hits.append((x, y))

            if hits:
                hit = min(hits, key=lambda p: math.hypot(p[0], p[1]))
                return DirectionRisk(
                    safe=False,
                    first_contact_deg=angle_abs,
                    hit_count=len(hits),
                    hit_xy=hit,
                    reason="PREDICTED_CONTACT",
                )

        return DirectionRisk(
            safe=True,
            first_contact_deg=None,
            hit_count=0,
            hit_xy=None,
            reason="CLEAR_TO_GUARD_ANGLE",
        )

    # ------------------------------------------------------------------
    # Command gating
    # ------------------------------------------------------------------
    def is_near_pivot(self, msg):
        return (
            abs(float(msg.linear.x)) <= self.args.pivot_linear_max
            and abs(float(msg.angular.z)) >= self.args.pivot_angular_min
        )

    def scan_fresh(self):
        return (
            self.scan_tf_ok
            and self.last_scan_mono > 0.0
            and (time.monotonic() - self.last_scan_mono)
            <= self.args.scan_timeout
        )

    def choose_risk(self, w):
        return self.left_risk if w > 0.0 else self.right_risk

    def publish_status(self, text):
        m = String()
        m.data = text
        self.status_pub.publish(m)

    def maybe_log(self, text, force=False):
        now = time.monotonic()
        changed = text != self.last_decision
        if force or changed or now - self.last_log_mono >= 1.0:
            self.get_logger().info(text)
            self.last_log_mono = now
            self.last_decision = text

    def cmd_cb(self, msg):
        self.cmd_count += 1

        v = float(msg.linear.x)
        w = float(msg.angular.z)

        # V1 only guards near-pivot commands.
        if not self.is_near_pivot(msg):
            decision = (
                f"PASS_NON_PIVOT profile={self.profile} v={v:+.3f} w={w:+.3f}"
            )
            self.publish_status(decision)
            self.maybe_log(decision)

            if self.args.active:
                self.out_pub.publish(msg)
            return

        # Fail closed for turn if LiDAR/TF is stale.
        if not self.scan_fresh():
            age = (
                time.monotonic() - self.last_scan_mono
                if self.last_scan_mono > 0.0
                else float("inf")
            )
            decision = (
                f"BLOCK_PIVOT_STALE_SCAN profile={self.profile} v={v:+.3f} w={w:+.3f} "
                f"scan_age={age:.3f}s"
            )
            self.block_count += 1
            self.publish_status(decision)
            self.maybe_log(decision, force=True)

            if self.args.active:
                self.out_pub.publish(Twist())
            return

        risk = self.choose_risk(w)
        direction = "LEFT" if w > 0.0 else "RIGHT"

        if not risk.safe:
            fc = (
                "NA"
                if risk.first_contact_deg is None
                else f"{risk.first_contact_deg:.1f}deg"
            )
            if risk.hit_xy is None:
                hit = "NA"
            else:
                hit = (
                    f"x={risk.hit_xy[0]:+.3f},"
                    f"y={risk.hit_xy[1]:+.3f}"
                )

            decision = (
                f"BLOCK_PIVOT_{direction} profile={self.profile} "
                f"v={v:+.3f} w={w:+.3f} "
                f"first_contact={fc} hit={hit} "
                f"reason={risk.reason}"
            )
            self.block_count += 1
            self.publish_status(decision)
            self.maybe_log(decision, force=True)

            if self.args.active:
                self.out_pub.publish(Twist())
            return

        decision = (
            f"ALLOW_PIVOT_{direction} profile={self.profile} "
            f"v={v:+.3f} w={w:+.3f} "
            f"clear_to={self.args.guard_angle_deg:.1f}deg"
        )
        self.publish_status(decision)
        self.maybe_log(decision)

        if self.args.active:
            self.out_pub.publish(msg)


def main():
    ap = argparse.ArgumentParser()

    mode = ap.add_mutually_exclusive_group()
    mode.add_argument(
        "--shadow",
        action="store_true",
        help="Observe/log only; send NO guarded command. Default.",
    )
    mode.add_argument(
        "--active",
        action="store_true",
        help="Publish guarded commands to output topic.",
    )

    ap.add_argument("--scan-topic", default="/scan")
    ap.add_argument("--base-frame", default="base_link")
    ap.add_argument("--input-topic", default="/cmd_vel_nav")
    ap.add_argument(
        "--output-topic",
        default="/cmd_vel_nav_guarded",
    )

    # Physical EMPTY footprint.
    ap.add_argument("--front", type=float, default=0.943)
    ap.add_argument("--rear", type=float, default=0.455)
    ap.add_argument("--left", type=float, default=0.370)
    ap.add_argument("--right", type=float, default=0.370)
    ap.add_argument("--padding", type=float, default=0.05)

    # FIX7 TEST LOADED footprint: intentionally identical to EMPTY.
    ap.add_argument("--loaded-front", type=float, default=0.943)
    ap.add_argument("--loaded-rear", type=float, default=0.455)
    ap.add_argument("--loaded-left", type=float, default=0.370)
    ap.add_argument("--loaded-right", type=float, default=0.370)
    ap.add_argument("--load-state-topic", default="/amr/load_state")

    # Protect the next 25 deg by default for the V3 active gate.
    ap.add_argument("--guard-angle-deg", type=float, default=25.0)
    ap.add_argument("--step-deg", type=float, default=1.0)

    # Near-pivot classification.
    ap.add_argument(
        "--pivot-linear-max",
        type=float,
        default=0.040,
        help="|v| <= this and |w| >= threshold => turn gate active",
    )
    ap.add_argument(
        "--pivot-angular-min",
        type=float,
        default=0.030,
    )

    ap.add_argument("--scan-timeout", type=float, default=0.35)
    ap.add_argument("--min-range", type=float, default=0.08)
    ap.add_argument("--max-range", type=float, default=8.0)

    args = ap.parse_args()

    # Shadow is the default unless --active is explicit.
    if not args.active:
        args.shadow = True

    if args.guard_angle_deg <= 0.0:
        raise SystemExit("--guard-angle-deg must be > 0")
    if args.step_deg <= 0.0:
        raise SystemExit("--step-deg must be > 0")
    if args.output_topic == args.input_topic and args.active:
        raise SystemExit(
            "REFUSED: active input and output topics are identical; "
            "this would create a feedback loop."
        )

    rclpy.init()
    node = TurnSafetyGate(args)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if args.active and node.out_pub is not None and rclpy.ok():
            # Best effort zero on shutdown, only while the ROS context is valid.
            node.out_pub.publish(Twist())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
