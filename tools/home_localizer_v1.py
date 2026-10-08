#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_srvs.srv import Trigger


class HomeLocalizer(Node):

    def __init__(self):
        super().__init__('amr_home_localizer')

        # ==========================================================
        # AUTHORITATIVE A_H - NEW ROUTE
        # ==========================================================
        self.home_x = 0.0702207104
        self.home_y = 0.0293381943
        self.home_yaw = 0.00022147645

        self.publisher = self.create_publisher(
            PoseWithCovarianceStamped,
            '/initialpose',
            10
        )

        self.service = self.create_service(
            Trigger,
            '/amr/reset_to_A_H',
            self.reset_service_callback
        )

        self.get_logger().info('======================================')
        self.get_logger().info(' AMR A_H HOME LOCALIZER V1')
        self.get_logger().info('======================================')
        self.get_logger().info(
            f'A_H: x={self.home_x:.3f} '
            f'y={self.home_y:.3f} '
            f'yaw={math.degrees(self.home_yaw):.3f} deg'
        )
        self.get_logger().info(
            'Service: /amr/reset_to_A_H'
        )

        # Chờ Nav2 / AMCL khởi động xong rồi tự reset A_H một lần.
        self.startup_timer = self.create_timer(
            3.0,
            self.startup_reset
        )

        self.startup_done = False

    def make_home_pose(self):
        msg = PoseWithCovarianceStamped()

        msg.header.frame_id = 'map'

        # stamp = 0 => TF dùng transform mới nhất.
        # Tránh lỗi extrapolation vài chục ms lúc startup.
        msg.header.stamp.sec = 0
        msg.header.stamp.nanosec = 0

        msg.pose.pose.position.x = self.home_x
        msg.pose.pose.position.y = self.home_y
        msg.pose.pose.position.z = 0.0

        half = self.home_yaw * 0.5

        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = math.sin(half)
        msg.pose.pose.orientation.w = math.cos(half)

        # Covariance:
        # XY std ≈ 2 cm
        # Yaw std ≈ 1 degree
        msg.pose.covariance = [0.0] * 36

        xy_var = 0.02 ** 2
        yaw_var = math.radians(1.0) ** 2

        msg.pose.covariance[0] = xy_var
        msg.pose.covariance[7] = xy_var
        msg.pose.covariance[35] = yaw_var

        return msg

    def publish_home(self):
        msg = self.make_home_pose()

        # Publish vài lần để tránh mất message lúc DDS vừa discovery.
        for _ in range(3):
            self.publisher.publish(msg)
            time.sleep(0.15)

        self.get_logger().info(
            'RESET A_H SENT: '
            f'x={self.home_x:.3f}, '
            f'y={self.home_y:.3f}, '
            f'yaw={math.degrees(self.home_yaw):.3f} deg'
        )

    def startup_reset(self):
        if self.startup_done:
            return

        # Chỉ tự reset khi AMCL đã subscribe /initialpose.
        if self.publisher.get_subscription_count() == 0:
            self.get_logger().warning(
                'Waiting for AMCL /initialpose subscriber...'
            )
            return

        self.publish_home()

        self.startup_done = True
        self.startup_timer.cancel()

        self.get_logger().info(
            'STARTUP A_H RESET COMPLETE'
        )

    def reset_service_callback(self, request, response):
        if self.publisher.get_subscription_count() == 0:
            response.success = False
            response.message = (
                'AMCL is not subscribed to /initialpose'
            )
            return response

        self.publish_home()

        response.success = True
        response.message = (
            f'A_H localization reset: '
            f'x={self.home_x:.3f}, '
            f'y={self.home_y:.3f}, '
            f'yaw={math.degrees(self.home_yaw):.3f} deg'
        )

        return response


def main():
    rclpy.init()

    node = HomeLocalizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
