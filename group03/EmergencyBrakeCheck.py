#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import Bool
from sensor_msgs.msg import Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class EmergencyBrakingNode(Node):
    def __init__(self):
        super().__init__("emergency_braking_node")
        self.get_logger().info("Emergency Braking Node started")

        # ——————————————————————————————————————
        # PARAMETERS
        self.declare_parameter("threshold_m", 0.1)
        self.threshold = self.get_parameter("threshold_m").value
        # ——————————————————————————————————————

        # state flag
        self.obstacle_detected = False

        # Publishers
        self.stop_publisher = self.create_publisher(Bool, "obstacle_detected", 10)
        self.cmd_pub = self.create_publisher(
            Twist, "/mirte_base_controller/cmd_vel", 10
        )

        # Subscriber to Nav2’s desired velocities
        self.cmd_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # Sonar subscriptions (BEST_EFFORT)
        self.create_subscription(
            Range,
            "/io/distance/front_left",
            self.front_left_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            "/io/distance/front_right",
            self.front_right_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            "/io/distance/rear_left",
            self.rear_left_callback,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Range,
            "/io/distance/rear_right",
            self.rear_right_callback,
            qos_profile_sensor_data,
        )

        # log the 4 readings every second
        self.create_timer(1.0, self.timer_callback)

        # hold the most recent ranges
        self.fl = self.fr = self.rl = self.rr = None

        # State Machine Subscription
        self.fsm_state = ""  # Holds latest FSM state
        self.create_subscription(String, "fsm_state", self.fsm_callback, 10)

    # FSM Callback
    def fsm_callback(self, msg: String):
        self.fsm_state = msg.data

    def cmd_vel_callback(self, msg: Twist):
        """Relay Nav2’s cmd_vel only if no obstacle is active."""
        if (
            self.fsm_state in ("NAV_TO_TREE", "NAV_TO_BASKET", "RETURNING_HOME")
            and not self.obstacle_detected
        ):
            self.cmd_pub.publish(msg)
            self.get_logger().info(f"Relaying cmd_vel: {msg.linear.x}, {msg.angular.z}")
        else:
            # override all velocity commands
            self.cmd_pub.publish(Twist())
            self.get_logger().info(
                "Obstacle detected or FSM state not suitable, stopping cmd_vel relay."
            )

    def publish_stop(self):
        if not self.obstacle_detected:
            self.obstacle_detected = True
            self.get_logger().warn(
                "Obstacle detected! Halting robot and stopping relay."
            )
            self.stop_publisher.publish(Bool(data=True))
        else:
            self.get_logger().warn("Obstacle already detected. No action taken.")

    def _process(self, label, msg: Range):
        """Shared logic: store, log, and check threshold."""
        val = msg.range
        setattr(self, label, val)

        # only check for obstacle
        if not math.isnan(val) and val < self.threshold:
            self.publish_stop()

    def front_left_callback(self, msg: Range):
        self._process("fl", msg)

    def front_right_callback(self, msg: Range):
        self._process("fr", msg)

    def rear_left_callback(self, msg: Range):
        self._process("rl", msg)

    def rear_right_callback(self, msg: Range):
        self._process("rr", msg)

    def timer_callback(self):
        fmt = lambda x: f"{x:.3f}" if (x is not None and not math.isnan(x)) else "n/a"
        self.get_logger().info(
            f"Ranges (m) | FL:{fmt(self.fl)} FR:{fmt(self.fr)} "
            f"RL:{fmt(self.rl)} RR:{fmt(self.rr)}  | Obstacle? {self.obstacle_detected}"
        )

        # --- reset obstacle_detected if all sensors are clear ---
        if self.obstacle_detected:
            below = [
                v
                for v in (self.fl, self.fr, self.rl, self.rr)
                if v is not None and not math.isnan(v) and v < self.threshold
            ]
            if not below:
                self.obstacle_detected = False
                self.get_logger().info("Obstacle cleared. Resuming relay.")
                self.stop_publisher.publish(Bool(data=False))


def main(args=None):
    rclpy.init(args=args)
    node = EmergencyBrakingNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
