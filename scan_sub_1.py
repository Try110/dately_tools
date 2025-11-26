#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserSub(Node):
    def __init__(self):
        super().__init__('laser_sub_demo')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.get_logger().info('节点已启动，等待 /scan 数据...')

    def listener_callback(self, msg: LaserScan):
        self.get_logger().info('收到激光数据！')


def main(args=None):
    rclpy.init(args=args)
    node = LaserSub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
