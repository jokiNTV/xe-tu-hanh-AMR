#!/usr/bin/env python3
"""
AMR ODOM ADAPTER V2.1
=====================

Purpose
-------
Prepare STM32 wheel odometry for robot_localization.

Input:
  /odom_raw

Output:
  /odom_ekf_input

The differential-drive chassis has a non-holonomic constraint:
  lateral body velocity vy ~= 0.

The STM32 raw odometry currently reports vy=0 but with covariance ~1e6,
which tells EKF to ignore that constraint. This adapter preserves the raw
message but gives the measured zero lateral velocity a realistic covariance.

No TF is published. No velocity command is published.
"""

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

INPUT_TOPIC = "/odom_raw"
OUTPUT_TOPIC = "/odom_ekf_input"

# [USER_TUNE]
# sigma(vy=0) = 0.02 m/s -> variance 0.0004
VY_ZERO_STDDEV_MPS = 0.02
VY_ZERO_VARIANCE = VY_ZERO_STDDEV_MPS ** 2


class OdomAdapter(Node):
    def __init__(self):
        super().__init__("amr_odom_adapter_v2_1")
        self.pub = self.create_publisher(Odometry, OUTPUT_TOPIC, 30)
        self.create_subscription(Odometry, INPUT_TOPIC, self.cb, 30)
        self.first = True
        self.get_logger().info(
            "ODOM ADAPTER V2.1 READY: /odom_raw -> /odom_ekf_input, "
            "vy=0 constraint enabled"
        )

    def cb(self, src: Odometry):
        msg = Odometry()
        msg.header = src.header
        msg.child_frame_id = src.child_frame_id
        msg.pose = src.pose
        msg.twist = src.twist

        # Differential-drive non-holonomic constraint.
        msg.twist.twist.linear.y = 0.0

        # Twist covariance index 7 = covariance(vy, vy)
        cov = list(msg.twist.covariance)
        cov[7] = VY_ZERO_VARIANCE
        msg.twist.covariance = cov

        self.pub.publish(msg)

        if self.first:
            self.first = False
            self.get_logger().info(
                "FIRST ODOM EKF INPUT: vx=%+.4f vy=%+.4f var_vy=%.6f"
                % (
                    msg.twist.twist.linear.x,
                    msg.twist.twist.linear.y,
                    VY_ZERO_VARIANCE,
                )
            )


def main():
    rclpy.init()
    node = OdomAdapter()
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
