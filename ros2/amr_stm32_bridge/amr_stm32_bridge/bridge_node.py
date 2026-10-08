"""ROS 2 Humble bridge for STM32F411 drive commands and odometry."""

from __future__ import annotations

from dataclasses import asdict, fields
import json
import math
import time

from geometry_msgs.msg import TransformStamped, Twist, Vector3Stamped
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
import serial
from std_msgs.msg import Empty, String
from std_srvs.srv import Trigger
from tf2_ros import TransformBroadcaster

from .kinematics import CommandLatch, twist_to_wheels
from .protocol import (
    DEVICE_DRIVE,
    DRIVE_CONFIG_GET,
    DRIVE_CONFIG_RESET,
    DRIVE_CONFIG_SET,
    DriveConfig,
    MODE_AUTO,
    MODE_MANUAL,
    MODE_SAFE,
    MSG_ACK,
    MSG_DRIVE_CONFIG,
    MSG_DRIVE_DIAGNOSTIC,
    MSG_NACK,
    MSG_ODOMETRY,
    ProtocolError,
    StreamDecoder,
    decode_ack,
    decode_drive_config,
    decode_drive_diagnostic,
    decode_odometry,
    make_clear_fault,
    make_drive_command,
    make_drive_config,
    make_set_mode,
    make_stream_control,
    make_turn_command,
    u32_is_newer,
)


STATE_DISCONNECTED = "DISCONNECTED"
STATE_SETTLING = "SETTLING"
STATE_WAIT_SAFE = "WAIT_SAFE_ACK"
STATE_WAIT_CLEAR = "WAIT_CLEAR_ACK"
STATE_WAIT_MODE = "WAIT_MODE_ACK"
STATE_WAIT_STREAM = "WAIT_STREAM_ACK"
STATE_READY = "READY"


def _pose_covariance(xy_stddev: float, yaw_stddev: float) -> list[float]:
    covariance = [0.0] * 36
    covariance[0] = xy_stddev * xy_stddev
    covariance[7] = xy_stddev * xy_stddev
    covariance[14] = 1.0e6
    covariance[21] = 1.0e6
    covariance[28] = 1.0e6
    covariance[35] = yaw_stddev * yaw_stddev
    return covariance


def _twist_covariance(
    linear_stddev: float,
    angular_stddev: float,
) -> list[float]:
    covariance = [0.0] * 36
    covariance[0] = linear_stddev * linear_stddev
    covariance[7] = 1.0e6
    covariance[14] = 1.0e6
    covariance[21] = 1.0e6
    covariance[28] = 1.0e6
    covariance[35] = angular_stddev * angular_stddev
    return covariance


class Stm32BridgeNode(Node):
    """Owns the USB CDC port, converts cmd_vel and publishes odometry."""

    def __init__(self) -> None:
        super().__init__("amr_stm32_bridge")

        self.declare_parameter("serial_port", "/dev/ttyACM0")
        self.declare_parameter("baud_rate", 115200)
        self.declare_parameter("drive_mode", MODE_AUTO)
        self.declare_parameter("clear_fault_on_connect", False)
        self.declare_parameter("stream_period_ms", 50)
        self.declare_parameter("include_drive_diagnostic", False)
        self.declare_parameter("reconnect_period_s", 2.0)
        self.declare_parameter("connect_settle_s", 0.25)
        self.declare_parameter("handshake_timeout_s", 0.75)
        self.declare_parameter("stream_timeout_s", 2.0)
        self.declare_parameter("command_rate_hz", 20.0)
        self.declare_parameter("cmd_vel_timeout_s", 0.25)
        self.declare_parameter("stm32_command_timeout_ms", 300)
        self.declare_parameter("drive_ack_every", 20)
        self.declare_parameter("wheel_radius_m", 0.10)
        self.declare_parameter("wheel_separation_m", 0.60)
        self.declare_parameter("max_wheel_speed_rad_s", 4.5)
        self.declare_parameter("cmd_vel_topic", "/cmd_vel")
        self.declare_parameter(
            "turn_command_topic",
            "/amr/drive/turn_relative",
        )
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter(
            "measured_wheel_speed_topic",
            "/amr/drive/measured_wheel_rad_s",
        )
        self.declare_parameter(
            "target_wheel_speed_topic",
            "/amr/drive/target_wheel_rad_s",
        )
        self.declare_parameter("wheel_pwm_topic", "/amr/drive/pwm")
        self.declare_parameter(
            "motion_profile_topic",
            "/amr/drive/motion_profile",
        )
        self.declare_parameter(
            "heading_state_topic",
            "/amr/drive/heading_state",
        )
        self.declare_parameter(
            "heading_rate_topic",
            "/amr/drive/heading_rate",
        )
        self.declare_parameter(
            "pid_config_command_topic",
            "/amr/pid_tuner/config_cmd",
        )
        self.declare_parameter(
            "pid_config_state_topic",
            "/amr/pid_tuner/config_state",
        )
        self.declare_parameter(
            "pid_stop_topic",
            "/amr/pid_tuner/stop",
        )
        self.declare_parameter("odom_frame", "odom")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("publish_tf", True)
        self.declare_parameter("pose_xy_stddev", 0.02)
        self.declare_parameter("pose_yaw_stddev", 0.03)
        self.declare_parameter("twist_linear_stddev", 0.03)
        self.declare_parameter("twist_angular_stddev", 0.05)

        self._load_parameters()

        odom_qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        command_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._odom_publisher = self.create_publisher(
            Odometry,
            self._odom_topic,
            odom_qos,
        )
        self._measured_wheel_speed_publisher = self.create_publisher(
            Vector3Stamped,
            self._measured_wheel_speed_topic,
            odom_qos,
        )
        self._target_wheel_speed_publisher = self.create_publisher(
            Vector3Stamped,
            self._target_wheel_speed_topic,
            odom_qos,
        )
        self._wheel_pwm_publisher = self.create_publisher(
            Vector3Stamped,
            self._wheel_pwm_topic,
            odom_qos,
        )
        self._motion_profile_publisher = self.create_publisher(
            Vector3Stamped,
            self._motion_profile_topic,
            odom_qos,
        )
        self._heading_state_publisher = self.create_publisher(
            Vector3Stamped,
            self._heading_state_topic,
            odom_qos,
        )
        self._heading_rate_publisher = self.create_publisher(
            Vector3Stamped,
            self._heading_rate_topic,
            odom_qos,
        )
        self._cmd_subscription = self.create_subscription(
            Twist,
            self._cmd_vel_topic,
            self._on_cmd_vel,
            command_qos,
        )
        self._turn_subscription = self.create_subscription(
            Vector3Stamped,
            self._turn_command_topic,
            self._on_turn_command,
            command_qos,
        )
        self._pid_config_state_publisher = self.create_publisher(
            String,
            self._pid_config_state_topic,
            odom_qos,
        )
        self._pid_config_subscription = self.create_subscription(
            String,
            self._pid_config_command_topic,
            self._on_pid_config_command,
            command_qos,
        )
        self._pid_stop_subscription = self.create_subscription(
            Empty,
            self._pid_stop_topic,
            self._on_pid_stop,
            command_qos,
        )
        self._clear_fault_service = self.create_service(
            Trigger,
            "/amr/clear_fault",
            self._on_clear_fault,
        )
        self._tf_broadcaster = (
            TransformBroadcaster(self) if self._publish_tf else None
        )

        self._pose_covariance = _pose_covariance(
            float(self.get_parameter("pose_xy_stddev").value),
            float(self.get_parameter("pose_yaw_stddev").value),
        )
        self._twist_covariance = _twist_covariance(
            float(self.get_parameter("twist_linear_stddev").value),
            float(self.get_parameter("twist_angular_stddev").value),
        )

        self._serial: serial.Serial | None = None
        self._decoder = StreamDecoder()
        self._sequence = int(time.monotonic() * 1000.0) & 0xFFFF
        self._state = STATE_DISCONNECTED
        self._pending_sequence: int | None = None
        self._pending_deadline_s = 0.0
        self._state_deadline_s = 0.0
        self._next_connect_s = 0.0
        self._last_odometry_s: float | None = None
        self._last_sample_count: int | None = None
        self._saw_invalid_odometry = False
        self._published_count = 0
        self._invalid_odometry_count = 0
        self._out_of_order_count = 0
        self._drive_send_count = 0
        self._drive_ack_pending: dict[int, float] = {}
        self._turn_deadline_s = 0.0
        self._turn_command_token = 0
        self._pid_config_pending: dict[int, str] = {}
        self._last_pid_config: DriveConfig | None = None
        self._pid_stop_guard_until_s = 0.0
        self._command_latch = CommandLatch(self._cmd_vel_timeout_s)
        self._command_was_stale = True
        self._last_error_log_s = 0.0
        self._last_limit_log_s = 0.0

        self._poll_timer = self.create_timer(0.005, self._poll)
        self._command_timer = self.create_timer(
            1.0 / self._command_rate_hz,
            self._send_periodic_drive_command,
        )

        self.get_logger().info(
            "Bridge configured for ROS 2 Humble: "
            f"{self._port_name}, cmd={self._cmd_vel_topic}, "
            f"odom={self._odom_topic}, wheel R={self._wheel_radius_m:.3f} m, "
            f"track={self._wheel_separation_m:.3f} m"
        )

    def _load_parameters(self) -> None:
        self._port_name = str(self.get_parameter("serial_port").value)
        self._baud_rate = int(self.get_parameter("baud_rate").value)
        self._drive_mode = int(self.get_parameter("drive_mode").value)
        self._clear_fault_on_connect = bool(
            self.get_parameter("clear_fault_on_connect").value
        )
        self._stream_period_ms = int(
            self.get_parameter("stream_period_ms").value
        )
        self._include_drive_diagnostic = bool(
            self.get_parameter("include_drive_diagnostic").value
        )
        self._reconnect_period_s = float(
            self.get_parameter("reconnect_period_s").value
        )
        self._connect_settle_s = float(
            self.get_parameter("connect_settle_s").value
        )
        self._handshake_timeout_s = float(
            self.get_parameter("handshake_timeout_s").value
        )
        self._stream_timeout_s = float(
            self.get_parameter("stream_timeout_s").value
        )
        self._command_rate_hz = float(
            self.get_parameter("command_rate_hz").value
        )
        self._cmd_vel_timeout_s = float(
            self.get_parameter("cmd_vel_timeout_s").value
        )
        self._stm32_command_timeout_ms = int(
            self.get_parameter("stm32_command_timeout_ms").value
        )
        self._drive_ack_every = int(
            self.get_parameter("drive_ack_every").value
        )
        self._wheel_radius_m = float(
            self.get_parameter("wheel_radius_m").value
        )
        self._wheel_separation_m = float(
            self.get_parameter("wheel_separation_m").value
        )
        self._max_wheel_speed_rad_s = float(
            self.get_parameter("max_wheel_speed_rad_s").value
        )
        self._cmd_vel_topic = str(
            self.get_parameter("cmd_vel_topic").value
        )
        self._turn_command_topic = str(
            self.get_parameter("turn_command_topic").value
        )
        self._odom_topic = str(self.get_parameter("odom_topic").value)
        self._measured_wheel_speed_topic = str(
            self.get_parameter("measured_wheel_speed_topic").value
        )
        self._target_wheel_speed_topic = str(
            self.get_parameter("target_wheel_speed_topic").value
        )
        self._wheel_pwm_topic = str(
            self.get_parameter("wheel_pwm_topic").value
        )
        self._motion_profile_topic = str(
            self.get_parameter("motion_profile_topic").value
        )
        self._heading_state_topic = str(
            self.get_parameter("heading_state_topic").value
        )
        self._heading_rate_topic = str(
            self.get_parameter("heading_rate_topic").value
        )
        self._pid_config_command_topic = str(
            self.get_parameter("pid_config_command_topic").value
        )
        self._pid_config_state_topic = str(
            self.get_parameter("pid_config_state_topic").value
        )
        self._pid_stop_topic = str(
            self.get_parameter("pid_stop_topic").value
        )
        self._odom_frame = str(self.get_parameter("odom_frame").value)
        self._base_frame = str(self.get_parameter("base_frame").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        if self._drive_mode not in (MODE_MANUAL, MODE_AUTO):
            raise ValueError("drive_mode must be MANUAL(1) or AUTO(2)")
        if not 20 <= self._stream_period_ms <= 1000:
            raise ValueError("stream_period_ms must be within [20, 1000]")
        if self._reconnect_period_s <= 0.0:
            raise ValueError("reconnect_period_s must be positive")
        if self._connect_settle_s < 0.0:
            raise ValueError("connect_settle_s cannot be negative")
        if self._handshake_timeout_s <= 0.0:
            raise ValueError("handshake_timeout_s must be positive")
        if self._stream_timeout_s <= 0.0:
            raise ValueError("stream_timeout_s must be positive")
        if self._command_rate_hz <= 0.0:
            raise ValueError("command_rate_hz must be positive")
        if not 0.05 <= self._cmd_vel_timeout_s <= 2.0:
            raise ValueError("cmd_vel_timeout_s must be within [0.05, 2.0]")
        if not 50 <= self._stm32_command_timeout_ms <= 1000:
            raise ValueError(
                "stm32_command_timeout_ms must be within [50, 1000]"
            )
        if self._drive_ack_every < 0:
            raise ValueError("drive_ack_every cannot be negative")
        if self._wheel_radius_m <= 0.0:
            raise ValueError("wheel_radius_m must be positive")
        if self._wheel_separation_m <= 0.0:
            raise ValueError("wheel_separation_m must be positive")
        if not 0.0 < self._max_wheel_speed_rad_s <= 4.5:
            raise ValueError("max_wheel_speed_rad_s must be within (0, 4.5]")

    def _next_sequence(self) -> int:
        self._sequence = (self._sequence + 1) & 0xFFFF
        if self._sequence == 0:
            self._sequence = 1
        return self._sequence

    def _connect(self) -> None:
        now = time.monotonic()
        if now < self._next_connect_s:
            return

        try:
            port = serial.Serial(
                self._port_name,
                self._baud_rate,
                timeout=0.0,
                write_timeout=0.2,
            )
            port.reset_input_buffer()
            port.reset_output_buffer()
        except (OSError, serial.SerialException) as exc:
            self._serial = None
            self._next_connect_s = now + self._reconnect_period_s
            self._log_error_throttled(
                f"Cannot open STM32 port {self._port_name}: {exc}"
            )
            return

        self._serial = port
        self._decoder.reset()
        self._command_latch.clear()
        self._command_was_stale = True
        self._pending_sequence = None
        self._last_odometry_s = None
        self._last_sample_count = None
        self._saw_invalid_odometry = False
        self._drive_ack_pending.clear()
        self._state = STATE_SETTLING
        self._state_deadline_s = now + self._connect_settle_s
        self.get_logger().info(f"Opened STM32 port: {self._port_name}")

    def _disconnect(self, reason: str) -> None:
        if self._serial is not None:
            try:
                self._serial.close()
            except (OSError, serial.SerialException):
                pass
        self._serial = None
        self._state = STATE_DISCONNECTED
        self._pending_sequence = None
        self._last_odometry_s = None
        self._last_sample_count = None
        self._saw_invalid_odometry = False
        self._drive_ack_pending.clear()
        self._pid_config_pending.clear()
        self._command_latch.clear()
        self._command_was_stale = True
        self._decoder.reset()
        self._next_connect_s = time.monotonic() + self._reconnect_period_s
        self.get_logger().warning(reason)

    def _write_frame(self, frame: bytes) -> bool:
        if self._serial is None:
            return False
        try:
            written = self._serial.write(frame)
            if written != len(frame):
                raise serial.SerialTimeoutException(
                    f"short serial write {written}/{len(frame)}"
                )
            return True
        except (OSError, serial.SerialException) as exc:
            self._disconnect(f"STM32 serial write failed: {exc}")
            return False

    def _wait_for_ack(self, state: str, sequence: int) -> None:
        self._state = state
        self._pending_sequence = sequence
        self._pending_deadline_s = (
            time.monotonic() + self._handshake_timeout_s
        )

    def _send_safe_mode(self) -> None:
        sequence = self._next_sequence()
        if self._write_frame(make_set_mode(sequence, MODE_SAFE)):
            self._wait_for_ack(STATE_WAIT_SAFE, sequence)

    def _send_drive_mode(self) -> None:
        sequence = self._next_sequence()
        if self._write_frame(make_set_mode(sequence, self._drive_mode)):
            self._wait_for_ack(STATE_WAIT_MODE, sequence)

    def _send_clear_fault(self) -> None:
        sequence = self._next_sequence()
        if self._write_frame(make_clear_fault(sequence)):
            self._wait_for_ack(STATE_WAIT_CLEAR, sequence)

    def _send_stream_control(self) -> None:
        sequence = self._next_sequence()
        frame = make_stream_control(
            sequence,
            self._stream_period_ms,
            include_drive_diagnostic=self._include_drive_diagnostic,
        )
        if self._write_frame(frame):
            self._wait_for_ack(STATE_WAIT_STREAM, sequence)

    def _poll(self) -> None:
        if self._serial is None:
            self._connect()
            return

        try:
            waiting = self._serial.in_waiting
            if waiting:
                data = self._serial.read(min(waiting, 4096))
                previous_errors = self._decoder.error_count
                previous_overflows = self._decoder.overflow_count
                for packet in self._decoder.feed(data):
                    self._handle_packet(packet)
                new_errors = self._decoder.error_count - previous_errors
                new_overflows = (
                    self._decoder.overflow_count - previous_overflows
                )
                if new_errors or new_overflows:
                    self._log_error_throttled(
                        "Serial frame rejected: "
                        f"decode={self._decoder.error_count}, "
                        f"overflow={self._decoder.overflow_count}"
                    )
        except (OSError, serial.SerialException) as exc:
            self._disconnect(f"STM32 serial link lost: {exc}")
            return

        if self._serial is None:
            return

        now = time.monotonic()
        if self._state == STATE_SETTLING:
            if now >= self._state_deadline_s:
                self._send_safe_mode()
            return

        if self._state in (
            STATE_WAIT_SAFE,
            STATE_WAIT_CLEAR,
            STATE_WAIT_MODE,
            STATE_WAIT_STREAM,
        ):
            if now > self._pending_deadline_s:
                self._disconnect(
                    f"STM32 handshake timeout in {self._state}"
                )
            return

        if self._state == STATE_READY:
            if (
                self._last_odometry_s is None
                or now - self._last_odometry_s > self._stream_timeout_s
            ):
                self._disconnect("Odometry stream timeout; reconnecting")

    def _handle_packet(self, packet) -> None:
        if packet.device_id != DEVICE_DRIVE:
            return
        if packet.message_id in (MSG_ACK, MSG_NACK):
            self._handle_ack_packet(packet)
            return
        if packet.message_id == MSG_DRIVE_CONFIG:
            self._handle_drive_config_packet(packet)
            return
        if packet.message_id == MSG_DRIVE_DIAGNOSTIC:
            self._handle_drive_diagnostic_packet(packet)
            return
        if packet.message_id != MSG_ODOMETRY:
            return

        self._last_odometry_s = time.monotonic()
        try:
            sample = decode_odometry(packet)
        except ProtocolError as exc:
            self._log_error_throttled(f"Invalid odometry frame: {exc}")
            return

        if not sample.valid:
            self._invalid_odometry_count += 1
            self._saw_invalid_odometry = True
            self._log_error_throttled("STM32 odometry is not valid")
            return

        if self._last_sample_count is not None:
            sample_is_newer = u32_is_newer(
                sample.sample_count,
                self._last_sample_count,
            )
            if not sample_is_newer:
                if self._saw_invalid_odometry:
                    # A hardware reset can leave USB CDC open while STM32
                    # restarts its odometry sample counter at zero. Rebaseline
                    # on the first valid sample after an invalid interval so
                    # the bridge recovers without a manual restart.
                    self.get_logger().warning(
                        "STM32 odometry counter restarted; rebaselining stream"
                    )
                    self._last_sample_count = None
                else:
                    self._out_of_order_count += 1
                    return
        self._saw_invalid_odometry = False
        self._last_sample_count = sample.sample_count
        self._publish_odometry(sample)

    def _publish_pid_status(self, status: str, **extra) -> None:
        payload = {"status": status}
        payload.update(extra)
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._pid_config_state_publisher.publish(message)

    def _publish_pid_config(self, config: DriveConfig) -> None:
        payload = asdict(config)
        payload["status"] = "OK"
        message = String()
        message.data = json.dumps(payload, sort_keys=True)
        self._pid_config_state_publisher.publish(message)

    def _request_pid_config(self) -> None:
        if self._serial is None or self._state != STATE_READY:
            return
        sequence = self._next_sequence()
        frame = make_drive_config(
            sequence,
            DRIVE_CONFIG_GET,
            self._last_pid_config or DriveConfig(),
            require_ack=False,
        )
        self._write_frame(frame)

    def _handle_drive_config_packet(self, packet) -> None:
        try:
            config = decode_drive_config(packet)
        except ProtocolError as exc:
            self._log_error_throttled(f"Invalid PID config report: {exc}")
            return
        self._last_pid_config = config
        self._publish_pid_config(config)

    def _on_pid_config_command(self, message: String) -> None:
        try:
            request = json.loads(message.data)
        except (TypeError, ValueError) as exc:
            self._publish_pid_status("ERROR", message=f"invalid JSON: {exc}")
            return

        if not isinstance(request, dict):
            self._publish_pid_status("ERROR", message="config command must be JSON object")
            return
        operation_name = str(request.get("op", "get")).strip().lower()

        if operation_name == "get":
            self._request_pid_config()
            return

        if self._serial is None or self._state != STATE_READY:
            self._publish_pid_status("ERROR", message="bridge is not READY")
            return

        if operation_name == "reset":
            operation = DRIVE_CONFIG_RESET
            config = self._last_pid_config or DriveConfig()
        elif operation_name == "set":
            operation = DRIVE_CONFIG_SET
            base = asdict(self._last_pid_config or DriveConfig())
            allowed = {field.name for field in fields(DriveConfig)}
            for key, value in request.items():
                if key == "op":
                    continue
                if key not in allowed:
                    self._publish_pid_status(
                        "ERROR", message=f"unknown PID field: {key}"
                    )
                    return
                base[key] = value
            try:
                config = DriveConfig(**base)
            except (TypeError, ValueError) as exc:
                self._publish_pid_status("ERROR", message=str(exc))
                return
        else:
            self._publish_pid_status(
                "ERROR", message=f"unsupported operation: {operation_name}"
            )
            return

        sequence = self._next_sequence()
        try:
            frame = make_drive_config(
                sequence, operation, config, require_ack=True
            )
        except (TypeError, ValueError) as exc:
            self._publish_pid_status("ERROR", message=str(exc))
            return

        if self._write_frame(frame):
            self._pid_config_pending[sequence] = operation_name
            self._publish_pid_status("PENDING", operation=operation_name)

    def _on_pid_stop(self, _message: Empty) -> None:
        if self._serial is None or self._state != STATE_READY:
            return
        now = time.monotonic()
        self._pid_stop_guard_until_s = now + 0.75
        self._turn_deadline_s = 0.0
        self._command_latch.clear()
        self._command_was_stale = True
        sequence = self._next_sequence()
        frame = make_drive_command(
            sequence,
            0.0,
            0.0,
            self._stm32_command_timeout_ms,
            require_ack=True,
        )
        if self._write_frame(frame):
            self._drive_ack_pending[sequence] = now
            self.get_logger().warning("PID tuner STOP: direct zero command sent")

    def _handle_drive_diagnostic_packet(self, packet) -> None:
        self._last_odometry_s = time.monotonic()

        try:
            sample = decode_drive_diagnostic(packet)
        except ProtocolError as exc:
            self._log_error_throttled(
                f"Invalid drive diagnostic frame: {exc}"
            )
            return

        stamp = self.get_clock().now().to_msg()
        self._publish_left_right_vector(
            self._measured_wheel_speed_publisher,
            sample.measured_left_rad_s,
            sample.measured_right_rad_s,
            stamp,
        )
        self._publish_left_right_vector(
            self._target_wheel_speed_publisher,
            sample.target_left_rad_s,
            sample.target_right_rad_s,
            stamp,
        )
        self._publish_left_right_vector(
            self._wheel_pwm_publisher,
            float(sample.pwm_left),
            float(sample.pwm_right),
            stamp,
        )
        self._publish_motion_profile(sample, stamp)
        self._publish_heading_diagnostics(sample, stamp)

    def _publish_motion_profile(self, sample, stamp) -> None:
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = self._base_frame
        message.vector.x = float(sample.motion_profile_status)
        message.vector.y = 1.0 if sample.motion_profile_fault else 0.0
        message.vector.z = 0.0
        self._motion_profile_publisher.publish(message)

    def _publish_heading_diagnostics(self, sample, stamp) -> None:
        state = Vector3Stamped()
        state.header.stamp = stamp
        state.header.frame_id = self._base_frame
        state.vector.x = sample.imu_yaw_rad
        state.vector.y = sample.heading_reference_rad
        state.vector.z = sample.heading_error_rad
        self._heading_state_publisher.publish(state)

        rate = Vector3Stamped()
        rate.header.stamp = stamp
        rate.header.frame_id = self._base_frame
        rate.vector.x = sample.gyro_z_rad_s
        rate.vector.y = sample.desired_yaw_rate_rad_s
        rate.vector.z = sample.heading_correction_rad_s
        self._heading_rate_publisher.publish(rate)

    def _publish_left_right_vector(
        self,
        publisher,
        left: float,
        right: float,
        stamp,
    ) -> None:
        message = Vector3Stamped()
        message.header.stamp = stamp
        message.header.frame_id = self._base_frame
        message.vector.x = left
        message.vector.y = right
        message.vector.z = 0.0
        publisher.publish(message)

    def _handle_ack_packet(self, packet) -> None:
        try:
            ack = decode_ack(packet)
        except ProtocolError as exc:
            self._log_error_throttled(f"Invalid ACK/NACK: {exc}")
            return

        if (
            self._pending_sequence is not None
            and ack.acknowledged_sequence == self._pending_sequence
        ):
            if ack.is_nack or ack.status != 0:
                self._disconnect(
                    f"STM32 rejected {self._state}: "
                    f"status={ack.status}, detail={ack.detail}"
                )
                return

            completed_state = self._state
            self._pending_sequence = None
            if (
                completed_state == STATE_WAIT_SAFE
                and self._clear_fault_on_connect
            ):
                self._send_clear_fault()
            elif completed_state in (STATE_WAIT_SAFE, STATE_WAIT_CLEAR):
                self._send_drive_mode()
            elif completed_state == STATE_WAIT_MODE:
                self._send_stream_control()
            elif completed_state == STATE_WAIT_STREAM:
                self._state = STATE_READY
                self._last_odometry_s = time.monotonic()
                self._command_latch.clear()
                self._command_was_stale = True
                self.get_logger().info(
                    "STM32 READY: SAFE reset -> drive mode -> odometry; "
                    f"publishing {self._odom_topic} and accepting "
                    f"{self._cmd_vel_topic}"
                )
                self._request_pid_config()
            return

        pid_operation = self._pid_config_pending.pop(
            ack.acknowledged_sequence,
            None,
        )
        if pid_operation is not None:
            if ack.is_nack or ack.status != 0:
                self._publish_pid_status(
                    "ERROR",
                    operation=pid_operation,
                    message=(
                        f"STM32 rejected PID config: status={ack.status}, "
                        f"detail={ack.detail}; stop the robot before APPLY"
                    ),
                )
            else:
                self._publish_pid_status("APPLIED", operation=pid_operation)
                self._request_pid_config()
            return

        sent_at = self._drive_ack_pending.pop(
            ack.acknowledged_sequence,
            None,
        )
        if sent_at is not None and (ack.is_nack or ack.status != 0):
            self._disconnect(
                "STM32 rejected drive command: "
                f"status={ack.status}, detail={ack.detail}"
            )

    def _on_cmd_vel(self, message: Twist) -> None:
        if not math.isfinite(message.linear.x) or not math.isfinite(
            message.angular.z
        ):
            self.get_logger().error("Rejected non-finite cmd_vel")
            return

        unsupported = (
            abs(message.linear.y) > 1.0e-6
            or abs(message.linear.z) > 1.0e-6
            or abs(message.angular.x) > 1.0e-6
            or abs(message.angular.y) > 1.0e-6
        )
        if unsupported:
            self._log_error_throttled(
                "Differential drive ignores cmd_vel components other than "
                "linear.x and angular.z"
            )

        now = time.monotonic()
        if now < self._pid_stop_guard_until_s:
            return

        command = twist_to_wheels(
            message.linear.x,
            message.angular.z,
            self._wheel_radius_m,
            self._wheel_separation_m,
            self._max_wheel_speed_rad_s,
        )
        self._command_latch.update(
            command.left_rad_s,
            command.right_rad_s,
            now,
        )

        if command.limited and now - self._last_limit_log_s >= 1.0:
            self.get_logger().warning(
                "cmd_vel scaled to preserve curvature: "
                f"scale={command.scale:.3f}, "
                f"left={command.left_rad_s:.3f}, "
                f"right={command.right_rad_s:.3f} rad/s"
            )
            self._last_limit_log_s = now

    def _on_turn_command(self, message: Vector3Stamped) -> None:
        """Forward one explicit relative turn to the STM32 controller.

        Vector fields are: x=relative angle [rad], y=max yaw rate [rad/s],
        z=timeout [s].  A turn has priority over stale/zero cmd_vel frames;
        a subsequent non-zero cmd_vel explicitly cancels it for safety.
        """
        relative_angle_rad = float(message.vector.x)
        max_yaw_rate_rad_s = float(message.vector.y)
        timeout_s = float(message.vector.z)

        if self._state != STATE_READY or self._serial is None:
            self.get_logger().warning("Rejected turn: STM32 bridge is not ready")
            return
        if not all(
            math.isfinite(value)
            for value in (
                relative_angle_rad,
                max_yaw_rate_rad_s,
                timeout_s,
            )
        ):
            self.get_logger().error("Rejected non-finite turn command")
            return
        if not -math.pi <= relative_angle_rad <= math.pi:
            self.get_logger().error("Rejected turn angle outside [-pi, pi]")
            return
        if not 0.12 <= max_yaw_rate_rad_s <= 0.50:
            self.get_logger().error(
                "Rejected turn max yaw rate outside [0.12, 0.50] rad/s"
            )
            return
        if not 1.0 <= timeout_s <= 15.0:
            self.get_logger().error(
                "Rejected turn timeout outside [1.0, 15.0] s"
            )
            return

        self._turn_command_token = (
            self._turn_command_token + 1
        ) & 0xFFFFFFFF
        sequence = self._next_sequence()
        timeout_ms = round(timeout_s * 1000.0)
        frame = make_turn_command(
            sequence,
            relative_angle_rad,
            max_yaw_rate_rad_s,
            timeout_ms,
            self._turn_command_token,
            require_ack=True,
        )
        if not self._write_frame(frame):
            return

        now = time.monotonic()
        self._drive_ack_pending[sequence] = now
        self._turn_deadline_s = now + timeout_s
        self._command_latch.clear()
        self._command_was_stale = True
        self.get_logger().info(
            "STM32 final-align turn started: "
            f"angle={math.degrees(relative_angle_rad):+.2f} deg, "
            f"rate<={max_yaw_rate_rad_s:.2f} rad/s, "
            f"timeout={timeout_s:.1f}s"
        )

    def _send_periodic_drive_command(self) -> None:
        if self._state != STATE_READY or self._serial is None:
            return

        now = time.monotonic()
        left_rad_s, right_rad_s, stale = self._command_latch.sample(now)

        if now < self._turn_deadline_s:
            command_is_nonzero = (
                abs(left_rad_s) > 1.0e-4 or abs(right_rad_s) > 1.0e-4
            )
            if stale or not command_is_nonzero:
                # Do not send periodic zero frames: on STM32 they would
                # replace and cancel the explicit turn command.
                return
            self._turn_deadline_s = 0.0
            self.get_logger().warning(
                "Non-zero cmd_vel overrides active STM32 final-align turn"
            )
        elif self._turn_deadline_s != 0.0:
            self._turn_deadline_s = 0.0
            self.get_logger().warning("STM32 final-align turn timed out")

        if stale and not self._command_was_stale:
            self.get_logger().warning(
                "cmd_vel timeout; commanding zero wheel speed"
            )
        self._command_was_stale = stale

        expired = [
            sequence
            for sequence, sent_at in self._drive_ack_pending.items()
            if now - sent_at > 2.0
        ]
        for sequence in expired:
            self._drive_ack_pending.pop(sequence, None)
        if expired:
            self._log_error_throttled(
                f"Missing {len(expired)} periodic drive ACK(s)"
            )

        self._drive_send_count += 1
        require_ack = (
            self._drive_ack_every > 0
            and self._drive_send_count % self._drive_ack_every == 0
        )
        sequence = self._next_sequence()
        frame = make_drive_command(
            sequence,
            left_rad_s,
            right_rad_s,
            self._stm32_command_timeout_ms,
            require_ack=require_ack,
        )
        if self._write_frame(frame) and require_ack:
            self._drive_ack_pending[sequence] = now

    def _publish_odometry(self, sample) -> None:
        stamp = self.get_clock().now().to_msg()
        half_yaw = 0.5 * sample.yaw_rad
        qz = math.sin(half_yaw)
        qw = math.cos(half_yaw)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame
        odom.child_frame_id = self._base_frame
        odom.pose.pose.position.x = sample.x_m
        odom.pose.pose.position.y = sample.y_m
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.pose.covariance = self._pose_covariance
        odom.twist.twist.linear.x = sample.linear_velocity_m_s
        odom.twist.twist.angular.z = sample.angular_velocity_rad_s
        odom.twist.covariance = self._twist_covariance
        self._odom_publisher.publish(odom)

        if self._tf_broadcaster is not None:
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = self._odom_frame
            transform.child_frame_id = self._base_frame
            transform.transform.translation.x = sample.x_m
            transform.transform.translation.y = sample.y_m
            transform.transform.rotation.z = qz
            transform.transform.rotation.w = qw
            self._tf_broadcaster.sendTransform(transform)

        self._published_count += 1
        if self._published_count == 1:
            self.get_logger().info(
                f"First valid odometry: {self._odom_topic}, "
                f"TF {self._odom_frame}->{self._base_frame}"
            )

    def _on_clear_fault(self, _request, response):
        if self._serial is None or self._state != STATE_READY:
            response.success = False
            response.message = "Bridge is not READY"
            return response

        self._command_latch.clear()
        self._send_clear_fault()
        if self._state != STATE_WAIT_CLEAR:
            response.success = False
            response.message = "Failed to send clear-fault request"
            return response
        response.success = True
        response.message = (
            "Clear-fault request sent; motion remains stopped until "
            "ACK and AUTO handshake complete"
        )
        return response

    def _log_error_throttled(self, message: str) -> None:
        now = time.monotonic()
        if now - self._last_error_log_s >= 2.0:
            self.get_logger().error(message)
            self._last_error_log_s = now

    def close(self) -> None:
        if self._serial is None:
            return

        try:
            self._serial.write(
                make_drive_command(
                    self._next_sequence(),
                    0.0,
                    0.0,
                    self._stm32_command_timeout_ms,
                    require_ack=False,
                )
            )
            self._serial.write(
                make_stream_control(
                    self._next_sequence(),
                    0,
                    include_drive_diagnostic=False,
                    require_ack=False,
                )
            )
            self._serial.write(
                make_set_mode(
                    self._next_sequence(),
                    MODE_SAFE,
                    require_ack=False,
                )
            )
            time.sleep(0.05)
        except (OSError, serial.SerialException):
            pass
        try:
            self._serial.close()
        except (OSError, serial.SerialException):
            pass
        self._serial = None
        self._state = STATE_DISCONNECTED


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
