#!/usr/bin/env python3
"""
AMR SCAN MID-STAMP V1.0
=======================

Diagnostic purpose:
- Keep the raw /scan completely unchanged for RViz / Collision Monitor.
- Republish an AMCL-only copy on /scan_amcl_mid.
- Move LaserScan.header.stamp from the driver's "start of capture" timestamp
  to the middle of scan_time.

Why:
Nav2 AMCL associates a LaserScan with one odom pose. For a rotating scanner,
the middle-of-scan pose is a less biased single-pose approximation than the
start-of-scan pose.

THIS IS NOT FULL DESKEW.
It is an A/B diagnostic to validate that scan timing is the localization root
cause before changing the SLLIDAR driver.

Input : /scan
Output: /scan_amcl_mid
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

INPUT_TOPIC = "/scan"
OUTPUT_TOPIC = "/scan_amcl_mid"


class ScanMidStamp(Node):
    def __init__(self):
        super().__init__("amr_scan_midstamp_v1_0")
        self.pub = self.create_publisher(
            LaserScan, OUTPUT_TOPIC, qos_profile_sensor_data
        )
        self.sub = self.create_subscription(
            LaserScan, INPUT_TOPIC, self.cb, qos_profile_sensor_data
        )
        self.first = True
        self.get_logger().info(
            "SCAN MID-STAMP READY: /scan -> /scan_amcl_mid"
        )

    def cb(self, src: LaserScan):
        # Create a clean copy.
        out = LaserScan()
        out.header = src.header
        out.angle_min = src.angle_min
        out.angle_max = src.angle_max
        out.angle_increment = src.angle_increment
        out.time_increment = src.time_increment
        out.scan_time = src.scan_time
        out.range_min = src.range_min
        out.range_max = src.range_max
        out.ranges = list(src.ranges)
        out.intensities = list(src.intensities)

        # Shift timestamp to middle of one revolution.
        ns = (
            int(src.header.stamp.sec) * 1_000_000_000
            + int(src.header.stamp.nanosec)
            + int(float(src.scan_time) * 0.5 * 1_000_000_000)
        )
        out.header.stamp.sec = ns // 1_000_000_000
        out.header.stamp.nanosec = ns % 1_000_000_000

        self.pub.publish(out)

        if self.first:
            self.first = False
            self.get_logger().info(
                "FIRST SCAN: N=%d scan_time=%.6fs shift=%.3fms"
                % (
                    len(out.ranges),
                    out.scan_time,
                    0.5 * out.scan_time * 1000.0,
                )
            )


def main():
    rclpy.init()
    node = ScanMidStamp()
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
