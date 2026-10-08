"""Differential-drive conversion and host command watchdog."""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class WheelCommand:
    left_rad_s: float
    right_rad_s: float
    scale: float

    @property
    def limited(self) -> bool:
        return self.scale < 1.0


def twist_to_wheels(
    linear_x_m_s: float,
    angular_z_rad_s: float,
    wheel_radius_m: float,
    wheel_separation_m: float,
    max_wheel_speed_rad_s: float,
) -> WheelCommand:
    values = (
        linear_x_m_s,
        angular_z_rad_s,
        wheel_radius_m,
        wheel_separation_m,
        max_wheel_speed_rad_s,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("kinematic input must be finite")
    if wheel_radius_m <= 0.0:
        raise ValueError("wheel radius must be positive")
    if wheel_separation_m <= 0.0:
        raise ValueError("wheel separation must be positive")
    if max_wheel_speed_rad_s <= 0.0:
        raise ValueError("wheel speed limit must be positive")

    half_track = 0.5 * wheel_separation_m
    left = (linear_x_m_s - angular_z_rad_s * half_track) / wheel_radius_m
    right = (linear_x_m_s + angular_z_rad_s * half_track) / wheel_radius_m
    peak = max(abs(left), abs(right))
    scale = 1.0
    if peak > max_wheel_speed_rad_s:
        scale = max_wheel_speed_rad_s / peak
        left *= scale
        right *= scale

    return WheelCommand(left_rad_s=left, right_rad_s=right, scale=scale)


class CommandLatch:
    """Stores the latest wheel target and returns zero after a timeout."""

    def __init__(self, timeout_s: float) -> None:
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("command timeout must be positive")
        self._timeout_s = timeout_s
        self.clear()

    def clear(self) -> None:
        self._left_rad_s = 0.0
        self._right_rad_s = 0.0
        self._received_at_s: float | None = None

    def update(self, left_rad_s: float, right_rad_s: float, now_s: float) -> None:
        if not all(math.isfinite(value) for value in (
            left_rad_s, right_rad_s, now_s
        )):
            raise ValueError("command latch input must be finite")
        self._left_rad_s = left_rad_s
        self._right_rad_s = right_rad_s
        self._received_at_s = now_s

    def sample(self, now_s: float) -> tuple[float, float, bool]:
        if not math.isfinite(now_s):
            raise ValueError("sample time must be finite")
        if self._received_at_s is None:
            return 0.0, 0.0, True
        if now_s - self._received_at_s > self._timeout_s:
            return 0.0, 0.0, True
        return self._left_rad_s, self._right_rad_s, False
