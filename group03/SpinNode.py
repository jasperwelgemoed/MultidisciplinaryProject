#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys


class SpinNode(Node):
    def __init__(self):
        super().__init__("spin_node")
        self.cmd_pub = self.create_publisher(
            Twist, "/mirte_base_controller/cmd_vel_unstamped", 10
        )  # '_unstamped' for gazebo only

        # Publish a spin command at 2 rad/s around z
        spin = Twist()
        spin.angular.z = 1.0
        self.get_logger().info("⟳ Starting spin: 2 circles at 1 rad/s")
        self.cmd_pub.publish(spin)

        # Schedule a one‐shot timer to stop after 3 circles (≈ 9.42 s)
        timer_period = (2 * 3.14159265 * 2) / 2.0  # (2π × 3) / 2 rad/s ≈ 9.4248 s
        self.create_timer(timer_period, self.stop_and_exit)

    def stop_and_exit(self):
        # Publish zero angular velocity to stop spinning
        stop = Twist()
        stop.angular.z = 0.0
        self.cmd_pub.publish(stop)
        self.get_logger().info("✅ Finished spinning 3 circles; stopping.")
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    node = SpinNode()
    rclpy.spin(node)
