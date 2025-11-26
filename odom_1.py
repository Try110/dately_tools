#!/usr/bin/env python3
import rclpy, math
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from transforms3d.euler import quat2euler, euler2quat


class OdomIntegrator(Node):
    def __init__(self):
        super().__init__('odom_integrator')

        # 参数
        self.declare_parameter('input_topic', '/odom')
        self.declare_parameter('output_topic', '/odom_smooth')
        self.declare_parameter('zupt_threshold', 0.02)  # 零速门限
        self.declare_parameter('lowpass_alpha', 1.0)  # 1.0=关滤波，0~1越小越平滑
        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.zupt_thresh = self.get_parameter('zupt_threshold').value
        self.alpha = self.get_parameter('lowpass_alpha').value

        # 状态
        self.x, self.y, self.yaw = 0.0, 0.0, 0.0
        self.last_time = self.get_clock().now()

        # ROS
        self.create_subscription(Odometry, self.input_topic, self.odom_cb, 10)
        self.pub = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

    # 低通滤波器
    def lowpass(self, prev, cur):
        return self.alpha * cur + (1.0 - self.alpha) * prev

    def odom_cb(self, msg: Odometry):
        if msg.twist.twist.angular.z > 0.000001:
            self.get_logger().info('odom callback ：msg.twist.twist.angular.z: %f' % msg.twist.twist.angular.z)
        return
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now

        # 提取速度
        vx_raw = msg.twist.twist.linear.x
        vy_raw = msg.twist.twist.linear.y
        w_raw = msg.twist.twist.angular.z
        self.get_logger().info('vx_raw : %f vy_raw : %f w_raw : %f' % (vx_raw, vy_raw, w_raw))
        # 零速更新（ZUPT）
        if abs(vx_raw) < self.zupt_thresh and abs(vy_raw) < self.zupt_thresh and abs(w_raw) < self.zupt_thresh:
            vx_raw = vy_raw = w_raw = 0.0
        self.get_logger().info('vx_raw : %f vy_raw : %f w_raw : %f' % (vx_raw, vy_raw, w_raw))
        # 低通滤波（可选）
        vx = self.lowpass(getattr(self, 'vx_prev', 0.0), vx_raw)
        vy = self.lowpass(getattr(self, 'vy_prev', 0.0), vy_raw)
        w = self.lowpass(getattr(self, 'w_prev', 0.0), w_raw)
        self.vx_prev, self.vy_prev, self.w_prev = vx, vy, w
        self.get_logger().info('vx: %f vy: %f w: %f' % (vx, vy, w))
        # 航位推算
        self.x += (vx * math.cos(self.yaw) - vy * math.sin(self.yaw)) * dt
        self.y += (vx * math.sin(self.yaw) + vy * math.cos(self.yaw)) * dt
        self.yaw += w * dt

        # 发布结果
        out = Odometry()
        out.header.stamp = now.to_msg()
        out.header.frame_id = msg.header.frame_id
        out.child_frame_id = msg.child_frame_id
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        qx, qy, qz, qw = euler2quat(0, 0, self.yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.twist = msg.twist  # 直接透传
        self.pub.publish(out)

        # TF
        t = TransformStamped()
        t.header.stamp = out.header.stamp
        t.header.frame_id = 'odom_new'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation = out.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main():
    rclpy.init()
    node = OdomIntegrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
