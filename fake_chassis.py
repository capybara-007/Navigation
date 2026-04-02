#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import math
from tf2_ros import TransformBroadcaster

class FakeChassis(Node):
    def __init__(self):
        super().__init__('fake_chassis')
        # 订阅 Nav2 发来的速度指令
        self.sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_cb, 10)
        # 发布里程计
        self.pub = self.create_publisher(Odometry, 'odom', 10)
        # 发布 TF
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.update) # 20Hz 刷新率
        
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.v = 0.0
        self.w = 0.0
        self.last_time = self.get_clock().now()
        self.get_logger().info("伪造底盘已启动！正在监听 /cmd_vel 并发布动态的 /odom 和 TF...")

    def cmd_cb(self, msg):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def update(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # 运动学积分计算当前位置
        self.x += self.v * math.cos(self.th) * dt
        self.y += self.v * math.sin(self.th) * dt
        self.th += self.w * dt

        # 发布动态 TF (odom -> base_link)
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.rotation.z = math.sin(self.th / 2.0)
        t.transform.rotation.w = math.cos(self.th / 2.0)
        self.br.sendTransform(t)

        # 发布 Odom 话题
        odom = Odometry()
        odom.header = t.header
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation = t.transform.rotation
        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w
        self.pub.publish(odom)

def main():
    rclpy.init()
    node = FakeChassis()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
