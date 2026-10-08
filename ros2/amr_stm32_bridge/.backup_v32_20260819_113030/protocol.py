"""AMR USB CDC protocol shared with the STM32F411 drive firmware.

Wire format:
COBS(header + payload + CRC16-CCITT-FALSE) followed by 0x00.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import struct
import time
from typing import Iterable


MAGIC = b"AM"
PROTOCOL_VERSION = 1
DEVICE_DRIVE = 0x10

MSG_ACK = 0x03
MSG_NACK = 0x04
MSG_CLEAR_FAULT = 0x06
MSG_SET_MODE = 0x07
MSG_ESTOP = 0x08
MSG_DRIVE_COMMAND = 0x20
MSG_ODOMETRY_STREAM_CONTROL = 0x26
MSG_ODOMETRY = 0x27
MSG_DRIVE_DIAGNOSTIC = 0x28

STREAM_OPTION_DRIVE_DIAGNOSTIC = 1 << 0

MODE_SAFE = 0
MODE_MANUAL = 1
MODE_AUTO = 2

FLAG_NONE = 0
FLAG_ACK_REQUIRED = 1 << 0

HEADER_FORMAT = "<2sBBBBHHI"
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
CRC_SIZE = 2
MAX_PAYLOAD_SIZE = 192
MAX_ENCODED_FRAME_SIZE = 256


class ProtocolError(ValueError):
    """Raised when a received AMR frame is malformed."""


@dataclass(frozen=True)
class Packet:
    device_id: int
    message_id: int
    flags: int
    sequence: int
    timestamp_ms: int
    payload: bytes


@dataclass(frozen=True)
class Ack:
    acknowledged_sequence: int
    status: int
    detail: int
    is_nack: bool


@dataclass(frozen=True)
class OdometrySample:
    sample_timestamp_ms: int
    x_m: float
    y_m: float
    yaw_rad: float
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    sample_count: int
    valid: bool


@dataclass(frozen=True)
class DriveDiagnosticSample:
    sample_timestamp_ms: int
    target_left_rad_s: float
    target_right_rad_s: float
    measured_left_rad_s: float
    measured_right_rad_s: float
    pwm_left: int
    pwm_right: int
    status: int
    mode: int
    sample_count: int
    imu_yaw_rad: float
    gyro_z_rad_s: float
    heading_reference_rad: float
    heading_error_rad: float
    heading_correction_rad_s: float
    desired_yaw_rate_rad_s: float
    yaw_rate_error_rad_s: float
    motion_profile_status: int
    motion_profile_fault: bool

    @property
    def imu_ready(self) -> bool:
        return bool(self.status & (1 << 2))

    @property
    def odometry_valid(self) -> bool:
        return bool(self.status & (1 << 3))


def crc16_ccitt_false(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    output = bytearray([0])
    code_index = 0
    code = 1

    for byte in data:
        if byte == 0:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1
            continue

        output.append(byte)
        code += 1
        if code == 0xFF:
            output[code_index] = code
            code_index = len(output)
            output.append(0)
            code = 1

    output[code_index] = code
    return bytes(output)


def cobs_decode(data: bytes) -> bytes:
    if not data:
        raise ProtocolError("empty COBS frame")

    output = bytearray()
    index = 0
    while index < len(data):
        code = data[index]
        if code == 0:
            raise ProtocolError("zero byte inside COBS frame")
        index += 1

        end = index + code - 1
        if end > len(data):
            raise ProtocolError("invalid COBS length")
        output.extend(data[index:end])
        index = end

        if code != 0xFF and index < len(data):
            output.append(0)

    return bytes(output)


def encode_packet(
    message_id: int,
    sequence: int,
    payload: bytes = b"",
    *,
    flags: int = FLAG_ACK_REQUIRED,
    device_id: int = DEVICE_DRIVE,
    timestamp_ms: int | None = None,
) -> bytes:
    if len(payload) > MAX_PAYLOAD_SIZE:
        raise ValueError("payload exceeds the STM32 protocol limit")
    if timestamp_ms is None:
        timestamp_ms = int(time.monotonic() * 1000) & 0xFFFFFFFF

    header = struct.pack(
        HEADER_FORMAT,
        MAGIC,
        PROTOCOL_VERSION,
        device_id,
        message_id,
        flags,
        sequence & 0xFFFF,
        len(payload),
        timestamp_ms & 0xFFFFFFFF,
    )
    content = header + payload
    crc = struct.pack("<H", crc16_ccitt_false(content))
    return cobs_encode(content + crc) + b"\x00"


def decode_frame(encoded_frame: bytes) -> Packet:
    raw = cobs_decode(encoded_frame)
    if len(raw) < HEADER_SIZE + CRC_SIZE:
        raise ProtocolError("frame is too short")

    content = raw[:-CRC_SIZE]
    received_crc = struct.unpack_from("<H", raw, len(raw) - CRC_SIZE)[0]
    expected_crc = crc16_ccitt_false(content)
    if received_crc != expected_crc:
        raise ProtocolError("CRC mismatch")

    (
        magic,
        version,
        device_id,
        message_id,
        flags,
        sequence,
        payload_length,
        timestamp_ms,
    ) = struct.unpack_from(HEADER_FORMAT, content, 0)

    if magic != MAGIC:
        raise ProtocolError("invalid magic")
    if version != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if payload_length > MAX_PAYLOAD_SIZE:
        raise ProtocolError("payload exceeds protocol limit")
    if len(content) != HEADER_SIZE + payload_length:
        raise ProtocolError("payload length mismatch")

    return Packet(
        device_id=device_id,
        message_id=message_id,
        flags=flags,
        sequence=sequence,
        timestamp_ms=timestamp_ms,
        payload=content[HEADER_SIZE:],
    )


def make_clear_fault(sequence: int, *, require_ack: bool = True) -> bytes:
    flags = FLAG_ACK_REQUIRED if require_ack else FLAG_NONE
    return encode_packet(MSG_CLEAR_FAULT, sequence, flags=flags)


def make_set_mode(
    sequence: int,
    mode: int,
    *,
    require_ack: bool = True,
) -> bytes:
    if mode not in (MODE_SAFE, MODE_MANUAL, MODE_AUTO):
        raise ValueError("unsupported drive mode")
    flags = FLAG_ACK_REQUIRED if require_ack else FLAG_NONE
    return encode_packet(MSG_SET_MODE, sequence, bytes([mode]), flags=flags)


def make_estop(sequence: int, *, require_ack: bool = True) -> bytes:
    flags = FLAG_ACK_REQUIRED if require_ack else FLAG_NONE
    return encode_packet(MSG_ESTOP, sequence, flags=flags)


def make_drive_command(
    sequence: int,
    left_rad_s: float,
    right_rad_s: float,
    timeout_ms: int,
    *,
    require_ack: bool = False,
) -> bytes:
    if not math.isfinite(left_rad_s) or not math.isfinite(right_rad_s):
        raise ValueError("wheel speed must be finite")
    if abs(left_rad_s) > 12.0 or abs(right_rad_s) > 12.0:
        raise ValueError("wheel speed exceeds protocol range")
    if not 50 <= timeout_ms <= 1000:
        raise ValueError("command timeout must be within [50, 1000] ms")

    payload = struct.pack(
        "<iiHH",
        round(left_rad_s * 1000.0),
        round(right_rad_s * 1000.0),
        timeout_ms,
        0,
    )
    flags = FLAG_ACK_REQUIRED if require_ack else FLAG_NONE
    return encode_packet(MSG_DRIVE_COMMAND, sequence, payload, flags=flags)


def make_stream_control(
    sequence: int,
    period_ms: int,
    *,
    include_drive_diagnostic: bool = False,
    require_ack: bool = True,
) -> bytes:
    if period_ms != 0 and not 20 <= period_ms <= 1000:
        raise ValueError("stream period must be 0 or within [20, 1000] ms")
    flags = FLAG_ACK_REQUIRED if require_ack else FLAG_NONE
    options = (
        STREAM_OPTION_DRIVE_DIAGNOSTIC
        if include_drive_diagnostic and period_ms != 0
        else 0
    )
    return encode_packet(
        MSG_ODOMETRY_STREAM_CONTROL,
        sequence,
        struct.pack("<HH", period_ms, options),
        flags=flags,
    )


def decode_ack(packet: Packet) -> Ack:
    if packet.message_id not in (MSG_ACK, MSG_NACK):
        raise ProtocolError("packet is not ACK/NACK")
    if len(packet.payload) != 4:
        raise ProtocolError("invalid ACK payload length")
    acknowledged, status, detail = struct.unpack("<HBB", packet.payload)
    return Ack(
        acknowledged_sequence=acknowledged,
        status=status,
        detail=detail,
        is_nack=packet.message_id == MSG_NACK,
    )


def decode_odometry(packet: Packet) -> OdometrySample:
    if packet.message_id != MSG_ODOMETRY:
        raise ProtocolError("packet is not odometry")
    if len(packet.payload) != 32:
        raise ProtocolError("invalid odometry payload length")

    (
        sample_timestamp_ms,
        x_mm,
        y_mm,
        yaw_mrad,
        linear_mm_s,
        angular_mrad_s,
        sample_count,
        status,
    ) = struct.unpack("<IiiiiiIB3x", packet.payload)

    return OdometrySample(
        sample_timestamp_ms=sample_timestamp_ms,
        x_m=x_mm * 0.001,
        y_m=y_mm * 0.001,
        yaw_rad=yaw_mrad * 0.001,
        linear_velocity_m_s=linear_mm_s * 0.001,
        angular_velocity_rad_s=angular_mrad_s * 0.001,
        sample_count=sample_count,
        valid=bool(status & 0x01),
    )


def decode_drive_diagnostic(packet: Packet) -> DriveDiagnosticSample:
    if packet.message_id != MSG_DRIVE_DIAGNOSTIC:
        raise ProtocolError("packet is not drive diagnostic")
    if len(packet.payload) != 112:
        raise ProtocolError("invalid drive diagnostic payload length")

    (
        sample_timestamp_ms,
        target_left_mrad_s,
        target_right_mrad_s,
        measured_left_mrad_s,
        measured_right_mrad_s,
        pwm_left,
        pwm_right,
    ) = struct.unpack_from("<Iiiiihh", packet.payload, 0)

    status = packet.payload[82]
    mode = packet.payload[83]
    sample_count = struct.unpack_from("<I", packet.payload, 84)[0]
    imu_yaw_mrad = struct.unpack_from("<i", packet.payload, 24)[0]
    gyro_z_mrad_s = struct.unpack_from("<i", packet.payload, 28)[0]
    heading_reference_mrad = struct.unpack_from("<i", packet.payload, 32)[0]
    heading_error_mrad = struct.unpack_from("<i", packet.payload, 36)[0]
    heading_correction_mrad_s = struct.unpack_from(
        "<i", packet.payload, 40
    )[0]
    desired_yaw_rate_mrad_s = struct.unpack_from(
        "<i", packet.payload, 88
    )[0]
    yaw_rate_error_mrad_s = struct.unpack_from(
        "<i", packet.payload, 92
    )[0]
    motion_profile_status = packet.payload[110]
    motion_profile_fault = bool(packet.payload[111])

    return DriveDiagnosticSample(
        sample_timestamp_ms=sample_timestamp_ms,
        target_left_rad_s=target_left_mrad_s * 0.001,
        target_right_rad_s=target_right_mrad_s * 0.001,
        measured_left_rad_s=measured_left_mrad_s * 0.001,
        measured_right_rad_s=measured_right_mrad_s * 0.001,
        pwm_left=pwm_left,
        pwm_right=pwm_right,
        status=status,
        mode=mode,
        sample_count=sample_count,
        imu_yaw_rad=imu_yaw_mrad * 0.001,
        gyro_z_rad_s=gyro_z_mrad_s * 0.001,
        heading_reference_rad=heading_reference_mrad * 0.001,
        heading_error_rad=heading_error_mrad * 0.001,
        heading_correction_rad_s=heading_correction_mrad_s * 0.001,
        desired_yaw_rate_rad_s=desired_yaw_rate_mrad_s * 0.001,
        yaw_rate_error_rad_s=yaw_rate_error_mrad_s * 0.001,
        motion_profile_status=motion_profile_status,
        motion_profile_fault=motion_profile_fault,
    )


def u32_is_newer(candidate: int, previous: int) -> bool:
    delta = (candidate - previous) & 0xFFFFFFFF
    return 0 < delta < 0x80000000


class StreamDecoder:
    """Incremental zero-delimited COBS decoder with automatic resync."""

    def __init__(self) -> None:
        self._encoded = bytearray()
        self._overflow = False
        self.error_count = 0
        self.overflow_count = 0

    def reset(self) -> None:
        self._encoded.clear()
        self._overflow = False

    def feed(self, data: bytes | bytearray | Iterable[int]) -> list[Packet]:
        packets: list[Packet] = []
        for byte in data:
            value = int(byte)
            if value != 0:
                if not self._overflow:
                    self._encoded.append(value)
                    if len(self._encoded) > MAX_ENCODED_FRAME_SIZE:
                        self._encoded.clear()
                        self._overflow = True
                        self.overflow_count += 1
                continue

            if self._overflow:
                self._overflow = False
                self._encoded.clear()
                continue
            if not self._encoded:
                continue

            try:
                packets.append(decode_frame(bytes(self._encoded)))
            except ProtocolError:
                self.error_count += 1
            finally:
                self._encoded.clear()

        return packets
