#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from rcl_interfaces.msg import Log


class NavigationNode(Node):
    def __init__(self):
        super().__init__("navigation_node")

        # Action client for NavigateToPose
        self._action_client = ActionClient(self, NavigateToPose, "/navigate_to_pose")

        # Subscriptions
        self.create_subscription(String, "/fsm_state", self.state_callback, 10)
        self.create_subscription(
            Odometry, "/mirte_base_controller/odom", self.set_odom, 10
        )
        self.create_subscription(Log, "/rosout", self.rosout_callback, 10)

        # Publisher to indicate navigation goal reached
        self._goal_reached_pub = self.create_publisher(Bool, "nav_goal_reached", 10)

        # store the current goal handle so we can cancel it
        self._current_goal_handle = None

        # Initialize variables
        self.current_pose = PoseStamped()
        self.current_pose.header.frame_id = "odom"

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.on_route = False
        self.goal_state = None

        self.get_logger().info("Navigation node started, waiting for FSM commands...")

        # Predefined goal poses by state name
        self.goals = {
            "NAV_TO_BASKET": self.create_pose(2.5, -3.2, 0.0),
            "NAV_TO_TREE": self.create_pose(0.0, -3.0, 0.0),
            "RETURNING_HOME": self.create_pose(0.0, 0.0, 0.0),
        }

    def set_odom(self, msg):
        self.current_pose.header = msg.header
        self.current_pose.pose = msg.pose.pose

    def create_pose(self, x, y, z):
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.position.z = z
        goal.pose.orientation.x = 0.0
        goal.pose.orientation.y = 0.0
        goal.pose.orientation.z = 0.0
        goal.pose.orientation.w = 1.0
        return goal

    def state_callback(self, msg: String):
        state = msg.data
        # self.get_logger().info(f"Received FSM state: '{state}'")

        if state not in self.goals:
            if self.on_route and self._current_goal_handle:
                self.get_logger().info(
                    "Non navigation state received: cancelling active navigation goal"
                )
                cancel_future = self._current_goal_handle.cancel_goal_async()
                cancel_future.add_done_callback(
                    lambda f: self.get_logger().info("Cancel request sent")
                )
                self.on_route = False
                self.goal_state = None
            else:
                self.get_logger().info(
                    "Non navigation state received but no active goal to cancel"
                )
                self.on_route = False
                return
        else:
            if self.on_route and self.goal_state == state:
                self.get_logger().info(
                    f"Already on route to goal for state '{state}', ignoring new command."
                )
                return
            else:
                self.on_route = True
                self.go_to_goal(state)
                self.get_logger().info(
                    f"Received FSM state: '{state}', navigating to goal..."
                )
                self.goal_state = state

    def go_to_goal(self, state):
        self.get_logger().info(f"Going to goal for state: {state}")
        goal_pose = self.goals[state]
        self.last_goal_pose = self.goals[state]

        # Wait for action server to be available
        self.get_logger().info("Waiting for /navigate_to_pose action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("NavigateToPose action server not available!")
            self.on_route = False
            return

        # Create goal message
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        send_goal_future = self._action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by action server")
            return
        self.get_logger().info("Goal accepted by action server")
        self._current_goal_handle = goal_handle

    def rosout_callback(self, msg: Log):
        if msg.name == "bt_navigator" and "Goal succeeded" in msg.msg:
            reached = Bool()
            reached.data = True
            self._goal_reached_pub.publish(reached)
            self.get_logger().info(
                "Detected bt_navigator Goal succeeded; published nav_goal_reached = True"
            )


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
