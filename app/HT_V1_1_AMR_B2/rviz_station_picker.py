#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class RvizStationPicker(Node):
    def __init__(self):
        super().__init__('rviz_station_picker')

        self.topic = '/amr/station_picker'
        self.subscription = self.create_subscription(
            PoseStamped,
            self.topic,
            self.pose_callback,
            10
        )

        self.get_logger().info(
            f'Waiting for RViz station poses on {self.topic}'
        )
        self.get_logger().info(
            'Use RViz 2D Goal Pose and publish to /amr/station_picker.'
        )

    def pose_callback(self, msg: PoseStamped):
        q = msg.pose.orientation

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw_deg = math.degrees(yaw)

        print()
        print('=' * 54)
        print('RVIZ STATION PICK')
        print(f'frame = {msg.header.frame_id}')
        print(f'x     = {x:.10f}')
        print(f'y     = {y:.10f}')
        print(f'yaw   = {yaw:.10f} rad')
        print(f'yaw   = {yaw_deg:.6f} deg')
        print('-' * 54)
        print('Copy-ready:')
        print(f'x: {x:.10f}')
        print(f'y: {y:.10f}')
        print(f'yaw: {yaw:.10f}')
        print('=' * 54)
        print(flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = RvizStationPicker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
