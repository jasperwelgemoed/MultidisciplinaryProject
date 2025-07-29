#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class FSMListener(Node):
    def __init__(self):
        super().__init__("listen_to_fsm_example")
        self.create_subscription(String, "fsm_state", self.state_cb, 10)
        self.get_logger().info("FSM Listener node started and subscribed to /fsm_state")

    def state_cb(self, msg):
        print(f"[FSM State Callback] Current state: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = FSMListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
