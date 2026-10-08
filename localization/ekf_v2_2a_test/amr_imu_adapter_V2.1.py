#!/usr/bin/env python3
"""
AMR IMU ADAPTER V2.1
====================

Inputs:
  /amr/drive/heading_state  vector.x = integrated MPU6050 yaw [rad]
  /amr/drive/heading_rate   vector.x = measured gyro Z [rad/s]

Output:
  /imu/data

Only yaw orientation and angular_velocity.z are declared useful.
robot_localization uses imu0_relative=true so STM32 boot yaw zero does not
need to match map yaw.
"""

import math
import time
import rclpy
from geometry_msgs.msg import Vector3Stamped
from rclpy.node import Node
from sensor_msgs.msg import Imu

HEADING_STATE_TOPIC = "/amr/drive/heading_state"
HEADING_RATE_TOPIC = "/amr/drive/heading_rate"
IMU_TOPIC = "/imu/data"
FRAME_ID = "base_link"

YAW_SIGN = +1.0
GYRO_SIGN = +1.0
YAW_STDDEV_RAD = math.radians(2.0)
GYRO_Z_STDDEV_RAD_S = 0.05
MAX_YAW_AGE_S = 0.20


def quat_from_yaw(yaw):
    h = 0.5 * yaw
    return 0.0, 0.0, math.sin(h), math.cos(h)


class ImuAdapter(Node):
    def __init__(self):
        super().__init__("amr_imu_adapter_v2_1")
        self.pub = self.create_publisher(Imu, IMU_TOPIC, 50)
        self.create_subscription(
            Vector3Stamped, HEADING_STATE_TOPIC, self.on_yaw, 50
        )
        self.create_subscription(
            Vector3Stamped, HEADING_RATE_TOPIC, self.on_rate, 50
        )
        self.last_yaw = None
        self.last_yaw_rx = None
        self.first = True
        self.get_logger().info(
            "IMU ADAPTER V2.1 READY: yaw + gyro_z -> /imu/data"
        )

    def on_yaw(self, msg):
        self.last_yaw = YAW_SIGN * float(msg.vector.x)
        self.last_yaw_rx = time.monotonic()

    def on_rate(self, msg):
        if self.last_yaw is None or self.last_yaw_rx is None:
            return
        if time.monotonic() - self.last_yaw_rx > MAX_YAW_AGE_S:
            return

        out = Imu()
        if msg.header.stamp.sec == 0 and msg.header.stamp.nanosec == 0:
            out.header.stamp = self.get_clock().now().to_msg()
        else:
            out.header.stamp = msg.header.stamp
        out.header.frame_id = FRAME_ID

        qx, qy, qz, qw = quat_from_yaw(self.last_yaw)
        out.orientation.x = qx
        out.orientation.y = qy
        out.orientation.z = qz
        out.orientation.w = qw

        yaw_var = YAW_STDDEV_RAD ** 2
        out.orientation_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, yaw_var,
        ]

        out.angular_velocity.z = GYRO_SIGN * float(msg.vector.x)
        gyro_var = GYRO_Z_STDDEV_RAD_S ** 2
        out.angular_velocity_covariance = [
            1e6, 0.0, 0.0,
            0.0, 1e6, 0.0,
            0.0, 0.0, gyro_var,
        ]

        out.linear_acceleration_covariance[0] = -1.0
        self.pub.publish(out)

        if self.first:
            self.first = False
            self.get_logger().info(
                "FIRST IMU: yaw=%+.2fdeg wz=%+.4frad/s"
                % (math.degrees(self.last_yaw), out.angular_velocity.z)
            )


def main():
    rclpy.init()
    node = ImuAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
