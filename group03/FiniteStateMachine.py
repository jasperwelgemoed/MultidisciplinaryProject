#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from enum import Enum
from std_msgs.msg import String, Bool
from std_msgs.msg import Int32


class RobotState(Enum):
    WAIT_FOR_COMMAND = 0  # F6 / Await farmer input
    STOPPED = 1  # Emergency stop (F7)
    RETURNING_HOME = 2  # Return to start (F2)
    PROCESS_ORDER = 3  # Check for remaining apples (F7)
    NAV_TO_TREE = 4  # Move to tree (F2)
    OBSTACLE_AVOIDANCE = 5  # Dynamic obstacle handling (F2.1)
    DETECTING_APPLES = 6  # Visual scan (F3)
    PICKING_APPLE = 7  # Grasping motion (F4)
    NAV_TO_BASKET = 8  # Navigate to basket (F2)
    PLACING_APPLE = 9  # Release motion (F5)
    # COMMUNICATING = 10  # Send status to farmer (F6)
    # EXCEPTION = 11  # Handle errors (F7)


class FSMNode(Node):
    def __init__(self):
        super().__init__("fsm_node")
        self.state = RobotState.WAIT_FOR_COMMAND  # Starting state

        # Subscriptions
        self.create_subscription(
            String, "farmer_cmd", self.farmer_callback, 10
        )  # 'stop', 'return_to_start', 'order_request'
        self.create_subscription(
            Bool, "obstacle_detected", self.obs_callback, 10
        )  # true / false
        self.create_subscription(
            Bool, "nav_goal_reached", self.nav_callback, 10
        )  # true / false
        self.create_subscription(
            Int32, "nr_of_apples_detected", self.apple_callback, 10
        )  # 0, 1, 2, 3, 4
        self.create_subscription(
            Bool, "gripper_above_base", self.grip_callback, 10
        )  # true / false

        # Publisher
        self.state_pub = self.create_publisher(String, "fsm_state", 10)
        self.timer = self.create_timer(1.0, self.loop)

        # Internal flags, all set to neutral / reset state
        self.obstacle = False
        self.nav_goal_reached = False
        self.nr_of_apples_detected = -1
        self.gripper_above_base = False
        self.previous_state = None
        self.farmer_command = ""
        self.apples_picked = 0  # Counter for picked apples
        self.required_apples = 4  # Number of apples to pick, set by farmer command

        self.get_logger().info(f"FSM initialized in {self.state.name}")

    def farmer_callback(self, msg: String):  # Make internal variable of farmer command
        cmd = msg.data.lower()
        if cmd not in ("stop", "return_to_start", "order_request"):
            self.get_logger().warn(f"Unknown command received: {cmd}")
            return
        self.farmer_command = cmd
        self.get_logger().info(f"Farmer command received: {cmd}")

    def obs_callback(
        self, msg: Bool
    ):  # Make internal variable of if obstacle is detected
        self.obstacle = msg.data

    def nav_callback(
        self, msg: Bool
    ):  # Make internal variable of if navigation goal is reached
        self.nav_goal_reached = msg.data

    def apple_callback(
        self, msg: Int32
    ):  # Make internal variable of number of apples detected
        self.nr_of_apples_detected = msg.data

    def grip_callback(
        self, msg: Bool
    ):  # Make internal variable of if gripper is done and above base again
        self.gripper_above_base = msg.data

    def loop(self):  # Main loop of the FSM
        state_msg = String(data=self.state.name)  # Publish current state
        self.state_pub.publish(state_msg)

        # Farmer command handling
        if self.farmer_command == "stop":
            self.state = RobotState.STOPPED
            self.farmer_command = ""
            self.get_logger().info("Emergency stop activated.")

        elif self.farmer_command == "return_to_start":
            self.get_logger().info("Returning to start...")
            self.state = RobotState.RETURNING_HOME
            self.farmer_command = ""
            self.nav_goal_reached = False  # reset flag

        elif self.farmer_command == "order_request":
            self.get_logger().info("Processing order...")
            self.state = RobotState.PROCESS_ORDER
            self.farmer_command = ""

        # Check for obstacle avoidance
        if (
            self.obstacle
            and (
                self.state == RobotState.NAV_TO_TREE
                or self.state == RobotState.NAV_TO_BASKET
                or self.state == RobotState.RETURNING_HOME
                or self.state == RobotState.PROCESS_ORDER
            )
            and self.state not in [RobotState.OBSTACLE_AVOIDANCE, RobotState.STOPPED]
        ):
            self.get_logger().info("Obstacle detected, switching to avoidance mode.")
            self.previous_state = self.state
            self.state = RobotState.OBSTACLE_AVOIDANCE

        # State handling
        if self.state == RobotState.WAIT_FOR_COMMAND:
            self.get_logger().info("Waiting for farmer command.")
            # if state == RobotState.WAIT_FOR_COMMAND: stop relay in dynamic_obstacles.py

        elif self.state == RobotState.STOPPED:
            self.get_logger().info("Robot stopped. Awaiting further instructions.")
            # stop mirte_base_controller by stopping relay in dynamic_obstacles.py

        elif self.state == RobotState.NAV_TO_TREE:
            if self.nav_goal_reached:
                self.nav_goal_reached = False  # reset flag
                self.get_logger().info("Reached tree.")
                self.state = RobotState.DETECTING_APPLES
                self.nr_of_apples_detected = -1  # reset
            else:
                self.get_logger().info("Navigating to tree...")

        elif self.state == RobotState.DETECTING_APPLES:
            if self.nr_of_apples_detected >= 1:
                self.get_logger().info(f"Detected {self.nr_of_apples_detected} apples.")
                self.nr_of_apples_detected = -1  # reset
                self.gripper_above_base = False  # reset flag
                self.state = RobotState.PICKING_APPLE
            elif self.nr_of_apples_detected == 0:
                self.get_logger().info("🍏 No apple detected.")
                self.nr_of_apples_detected = -1  # reset
                self.state = RobotState.RETURNING_HOME
                self.nav_goal_reached = False  # reset flag
            else:
                self.get_logger().info("Waiting for apple detection...")

        elif self.state == RobotState.PICKING_APPLE:
            if self.gripper_above_base:
                self.get_logger().info("Apple picked, navigating to basket.")
                self.state = RobotState.NAV_TO_BASKET
                self.nav_goal_reached = False  # reset flag
            else:
                self.get_logger().info("Waiting for gripper to return above base...")

        elif self.state == RobotState.NAV_TO_BASKET:
            if self.nav_goal_reached:
                self.gripper_above_base = False
                self.get_logger().info("Reached basket.")
                self.state = RobotState.PLACING_APPLE
            else:
                self.get_logger().info("Navigating to basket...")

        elif self.state == RobotState.PLACING_APPLE:
            if self.gripper_above_base:
                self.get_logger().info("Apple placed. Returning to process order state")
                self.state = RobotState.PROCESS_ORDER
            else:
                self.get_logger().info("Waiting for gripper to return above base...")

        elif self.state == RobotState.OBSTACLE_AVOIDANCE:
            if not self.obstacle:
                self.get_logger().info(
                    f"Obstacle cleared, returning to {self.previous_state}"
                )
                self.state = self.previous_state
            else:
                self.get_logger().info("Avoiding obstacle...")

        elif self.state == RobotState.RETURNING_HOME:
            if self.nav_goal_reached:
                self.get_logger().info("Reached home.")
                self.previous_state = RobotState.RETURNING_HOME
                # self.state = RobotState.COMMUNICATING #After it has returned to home, we don't want it to go back in the field automatically.
                self.state = RobotState.WAIT_FOR_COMMAND
            else:
                self.get_logger().info("Returning to start...")

        # elif self.state == RobotState.COMMUNICATING:
        #     self.get_logger().info("Communicating status to farmer...")
        #     self.get_logger().info(
        #         f"State to communicate to farmer: {self.previous_state.name}"
        #     )

        #     if self.previous_state == RobotState.RETURNING_HOME:
        #         self.get_logger().info("All apples picked. Returning home.")
        #         self.state = RobotState.RETURNING_HOME
        #     else:
        #         self.state = RobotState.PROCESS_ORDER

        # elif self.state == RobotState.EXCEPTION:
        #     self.get_logger().info("Handling exception...")
        #     # TODO: error recovery or escalate
        #     self.previous_state = RobotState.EXCEPTION
        #     self.state = RobotState.COMMUNICATING

        elif self.state == RobotState.PROCESS_ORDER:
            self.get_logger().info("Processing order...")
            self.nav_goal_reached = False
            self.nr_of_apples_detected = -1
            self.gripper_above_base = False
            self.previous_state = None
            if self.required_apples > self.apples_picked:
                self.get_logger().info(
                    f"Apples picked: {self.apples_picked}, required: {self.required_apples}"
                )
                self.state = RobotState.NAV_TO_TREE
            else:
                self.get_logger().info("No order request, returning to wait state.")
                self.state = RobotState.WAIT_FOR_COMMAND

        else:
            self.get_logger().warn("Unknown state, resetting to WAIT_FOR_COMMAND.")
            self.state = RobotState.WAIT_FOR_COMMAND


def main(args=None):
    rclpy.init(args=args)
    node = FSMNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


# #!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from enum import Enum
# from std_msgs.msg import String, Bool
# from std_msgs.msg import Int32


# class RobotState(Enum):
#     WAIT_FOR_COMMAND = 0  # F6 / Await farmer input
#     STOPPED = 1  # Emergency stop (F7)
#     RETURNING_HOME = 2  # Return to start (F2)
#     PROCESS_ORDER = 3  # Check for remaining apples (F7)
#     NAV_TO_TREE = 4  # Move to tree (F2)
#     OBSTACLE_AVOIDANCE = 5  # Dynamic obstacle handling (F2.1)
#     DETECTING_APPLES = 6  # Visual scan (F3)
#     PICKING_APPLE = 7  # Grasping motion (F4)
#     NAV_TO_BASKET = 8  # Navigate to basket (F2)
#     PLACING_APPLE = 9  # Release motion (F5)
#     COMMUNICATING = 10  # Send status to farmer (F6)
#     EXCEPTION = 11  # Handle errors (F7)


# class FSMNode(Node):
#     def __init__(self):
#         super().__init__("fsm")
#         self.state = RobotState.WAIT_FOR_COMMAND  # Starting state

#         # Subscriptions
#         self.create_subscription(
#             String, "farmer_cmd", self.farmer_callback, 10
#         )  # 'stop', 'return_to_start', 'order_request'
#         self.create_subscription(
#             Bool, "obstacle_detected", self.obs_callback, 10
#         )  # true / false
#         self.create_subscription(
#             Bool, "nav_goal_reached", self.nav_callback, 10
#         )  # true / false
#         self.create_subscription(
#             Int32, "nr_of_apples_detected", self.apple_callback, 10
#         )  # 0, 1, 2, 3, 4
#         self.create_subscription(
#             Bool, "gripper_above_base", self.grip_callback, 10
#         )  # true / false

#         # Publisher
#         self.state_pub = self.create_publisher(String, "fsm_state", 10)
#         self.timer = self.create_timer(0.1, self.loop)

#         # Internal flags, all set to neutral / reset state
#         self.obstacle = False
#         self.nav_goal_reached = False
#         self.nr_of_apples_detected = -1
#         self.gripper_above_base = False
#         self.previous_state = None
#         self.farmer_command = ""

#         self.get_logger().info(f"FSM initialized in {self.state.name}")

#     def farmer_callback(self, msg: String):  # Make internal variable of farmer command
#         cmd = msg.data.lower()
#         if cmd not in ("stop", "return_to_start", "order_request"):
#             self.get_logger().warn(f"Unknown command received: {cmd}")
#             return
#         self.farmer_command = cmd
#         self.get_logger().info(f"Farmer command received: {cmd}")

#     def obs_callback(
#         self, msg: Bool
#     ):  # Make internal variable of if obstacle is detected
#         self.obstacle = msg.data

#     def nav_callback(
#         self, msg: Bool
#     ):  # Make internal variable of if navigation goal is reached
#         self.nav_goal_reached = msg.data

#     def apple_callback(
#         self, msg: Bool
#     ):  # Make internal variable of number of apples detected
#         self.nr_of_apples_detected = msg.data

#     def grip_callback(
#         self, msg: Bool
#     ):  # Make internal variable of if gripper is done and above base again
#         self.gripper_above_base = msg.data

#     def loop(self):  # Main loop of the FSM
#         state_msg = String(data=self.state.name)  # Publish current state
#         self.state_pub.publish(state_msg)

#         # Farmer command handling
#         if self.farmer_command == "stop":
#             self.state = RobotState.STOPPED
#             self.farmer_command = ""
#             self.get_logger().info("Emergency stop activated.")

#         elif self.farmer_command == "return_to_start":
#             self.get_logger().info("Returning to start...")
#             self.state = RobotState.RETURNING_HOME
#             self.farmer_command = ""

#         elif self.farmer_command == "order_request":
#             self.get_logger().info("Processing order...")
#             self.state = RobotState.NAV_TO_TREE
#             self.farmer_command = ""

#         # Check for obstacle avoidance
#         if (
#             self.obstacle
#             and (
#                 self.state == RobotState.NAV_TO_TREE
#                 or self.state == RobotState.NAV_TO_BASKET
#                 or self.state == RobotState.RETURNING_HOME
#             )
#             and self.state not in [RobotState.OBSTACLE_AVOIDANCE, RobotState.STOPPED]
#         ):
#             self.get_logger().info("Obstacle detected, switching to avoidance mode.")
#             self.previous_state = self.state
#             self.state = RobotState.OBSTACLE_AVOIDANCE

#         # State handling
#         if self.state == RobotState.WAIT_FOR_COMMAND:
#             self.get_logger().info("Waiting for farmer command.")
#             # if state == RobotState.WAIT_FOR_COMMAND: stop relay in dynamic_obstacles.py

#         elif self.state == RobotState.STOPPED:
#             self.get_logger().info("Robot stopped. Awaiting further instructions.")
#             # stop mirte_base_controller by stopping relay in dynamic_obstacles.py

#         elif self.state == RobotState.NAV_TO_TREE:
#             if self.nav_goal_reached:
#                 self.nav_goal_reached = False  # reset flag
#                 self.get_logger().info("Reached tree.")
#                 self.state = RobotState.DETECTING_APPLES
#             else:
#                 self.get_logger().info("Navigating to tree...")
#                 # Trigger navigation to tree
#                 # publish goal location to nav2
#                 # or use a service to set the goal
#                 # For example, publish a new goal to the navigation stack
#                 # or use a service to set the goal

#         elif self.state == RobotState.DETECTING_APPLES:
#             self.get_logger().info(f"Scanning for apples... ")

#             if self.nr_of_apples_detected >= 1:
#                 self.get_logger().info(f"Detected {self.nr_of_apples_detected} apples.")
#                 self.nr_of_apples_detected = -1  # reset
#                 self.state = RobotState.PICKING_APPLE
#             elif self.nr_of_apples_detected == 0:
#                 self.get_logger().info("🍏 No apple detected.")
#                 self.nr_of_apples_detected = -1  # reset
#                 self.state = RobotState.RETURNING_HOME

#         elif self.state == RobotState.PICKING_APPLE:
#             self.get_logger().info("Picking apple...")
#             if self.gripper_above_base:
#                 self.get_logger().info("Gripper full, navigating to basket.")
#                 self.state = RobotState.NAV_TO_BASKET

#         elif self.state == RobotState.NAV_TO_BASKET:
#             if self.nav_goal_reached:
#                 self.nav_goal_reached = False  # reset flag
#                 self.get_logger().info("Reached basket.")
#                 self.state = RobotState.PLACING_APPLE
#             else:
#                 self.get_logger().info("Navigating to basket...")
#                 # Trigger navigation to basket
#                 # publish goal location to nav2

#         elif self.state == RobotState.PLACING_APPLE:
#             self.get_logger().info("Placing apple in basket...")
#             if not self.gripper_above_base:
#                 self.get_logger().info("Apple placed. Returning to process order state")
#                 self.state = RobotState.PROCESS_ORDER

#         elif self.state == RobotState.OBSTACLE_AVOIDANCE:
#             if not self.obstacle:
#                 self.get_logger().info(
#                     f"Obstacle cleared, returning to {self.previous_state}"
#                 )
#                 self.state = self.previous_state
#             else:
#                 self.get_logger().info("Avoiding obstacle...")

#         elif self.state == RobotState.RETURNING_HOME:
#             if self.obstacle:
#                 self.state = RobotState.OBSTACLE_AVOIDANCE
#                 self.get_logger().info(
#                     "Obstacle detected, switching to avoidance mode."
#                 )
#             elif self.nav_goal_reached:
#                 self.nav_goal_reached = False
#                 self.get_logger().info("Reached home.")
#                 self.previous_state = RobotState.RETURNING_HOME
#                 # self.state = RobotState.COMMUNICATING #After it has returned to home, we don't want it to go back in the field automatically.
#                 self.state = RobotState.WAIT_FOR_COMMAND
#             else:
#                 self.get_logger().info("Returning to start...")
#                 # Trigger navigation to home
#                 # publish goal location to nav2

#         elif self.state == RobotState.COMMUNICATING:
#             self.get_logger().info("Communicating status to farmer...")
#             self.get_logger().info(
#                 f"State to communicate to farmer: {self.previous_state.name}"
#             )

#             if self.previous_state == RobotState.RETURNING_HOME:
#                 self.get_logger().info("All apples picked. Returning home.")
#                 self.state = RobotState.RETURNING_HOME
#             else:
#                 self.state = RobotState.PROCESS_ORDER

#         elif self.state == RobotState.EXCEPTION:
#             self.get_logger().info("Handling exception...")
#             # TODO: error recovery or escalate
#             self.previous_state = RobotState.EXCEPTION
#             self.state = RobotState.COMMUNICATING

#         else:
#             self.get_logger().warn("Unknown state, resetting to WAIT_FOR_COMMAND.")
#             self.state = RobotState.WAIT_FOR_COMMAND


# def main(args=None):
#     rclpy.init(args=args)
#     node = FSMNode()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()

# # #!/usr/bin/env python3
# # import rclpy
# # from rclpy.node import Node
# # from enum import Enum
# # from std_msgs.msg import String, Bool
# # from std_msgs.msg import Int32


# # class RobotState(Enum):
# #     WAIT_FOR_COMMAND = 0  # F6 / Await farmer input
# #     STOPPED = 1  # Emergency stop (F7)
# #     RETURNING_HOME = 2  # Return to start (F2)
# #     PROCESS_ORDER = 3  # Check for remaining apples (F7)
# #     NAV_TO_TREE = 4  # Move to tree (F2)
# #     OBSTACLE_AVOIDANCE = 5  # Dynamic obstacle handling (F2.1)
# #     DETECTING_APPLES = 6  # Visual scan (F3)
# #     PICKING_APPLE = 7  # Grasping motion (F4)
# #     NAV_TO_BASKET = 8  # Navigate to basket (F2)
# #     PLACING_APPLE = 9  # Release motion (F5)
# #     COMMUNICATING = 10  # Send status to farmer (F6)
# #     EXCEPTION = 11  # Handle errors (F7)


# # class FSMNode(Node):
# #     def __init__(self):
# #         super().__init__("fsm")
# #         self.state = RobotState.WAIT_FOR_COMMAND  # Starting state

# #         # Subscriptions
# #         self.create_subscription(
# #             String, "farmer_cmd", self.farmer_callback, 10
# #         )  # 'stop', 'return_to_start', 'order_request'
# #         self.create_subscription(
# #             Bool, "obstacle_detected", self.obs_callback, 10
# #         )  # true / false
# #         self.create_subscription(
# #             Bool, "nav_goal_reached", self.nav_callback, 10
# #         )  # true / false
# #         self.create_subscription(
# #             Int32, "nr_of_apples_detected", self.apple_callback, 10
# #         )  # 0, 1, 2, 3, 4
# #         self.create_subscription(
# #             Bool, "gripper_above_base", self.grip_callback, 10
# #         )  # true / false

# #         # Publisher
# #         self.state_pub = self.create_publisher(String, "fsm_state", 10)
# #         self.timer = self.create_timer(0.1, self.loop)

# #         # Internal flags, all set to neutral / reset state
# #         self.obstacle = False
# #         self.nav_goal_reached = False
# #         self.nr_of_apples_detected = -1
# #         self.gripper_above_base = False
# #         self.previous_state = None
# #         self.farmer_command = ""

# #         self.get_logger().info(f"FSM initialized in {self.state.name}")

# #     def farmer_callback(self, msg: String):  # Make internal variable of farmer command
# #         cmd = msg.data.lower()
# #         if cmd not in ("stop", "return_to_start", "order_request"):
# #             self.get_logger().warn(f"Unknown command received: {cmd}")
# #             return
# #         self.farmer_command = cmd
# #         self.get_logger().info(f"Farmer command received: {cmd}")

# #     def obs_callback(
# #         self, msg: Bool
# #     ):  # Make internal variable of if obstacle is detected
# #         self.obstacle = msg.data

# #     def nav_callback(
# #         self, msg: Bool
# #     ):  # Make internal variable of if navigation goal is reached
# #         self.nav_goal_reached = msg.data

# #     def apple_callback(
# #         self, msg: Bool
# #     ):  # Make internal variable of number of apples detected
# #         self.nr_of_apples_detected = msg.data

# #     def grip_callback(
# #         self, msg: Bool
# #     ):  # Make internal variable of if gripper is done and above base again
# #         self.gripper_above_base = msg.data

# #     def loop(self):  # Main loop of the FSM
# #         state_msg = String(data=self.state.name)  # Publish current state
# #         self.state_pub.publish(state_msg)

# #         # Farmer command handling
# #         if self.farmer_command == "stop":
# #             self.state = RobotState.STOPPED
# #             self.farmer_command = ""
# #             self.get_logger().info("Emergency stop activated.")

# #         elif self.farmer_command == "return_to_start":
# #             self.get_logger().info("Returning to start...")
# #             self.state = RobotState.RETURNING_HOME
# #             self.farmer_command = ""

# #         elif self.farmer_command == "order_request":
# #             self.get_logger().info("Processing order...")
# #             self.state = RobotState.NAV_TO_TREE
# #             self.farmer_command = ""

# #         # Check for obstacle avoidance
# #         if (
# #             self.obstacle
# #             and (
# #                 self.state == RobotState.NAV_TO_TREE
# #                 or self.state == RobotState.NAV_TO_BASKET
# #                 or self.state == RobotState.RETURNING_HOME
# #             )
# #             and self.state not in [RobotState.OBSTACLE_AVOIDANCE, RobotState.STOPPED]
# #         ):
# #             self.get_logger().info("Obstacle detected, switching to avoidance mode.")
# #             self.previous_state = self.state
# #             self.state = RobotState.OBSTACLE_AVOIDANCE

# #         # State handling
# #         if self.state == RobotState.WAIT_FOR_COMMAND:
# #             self.get_logger().info("Waiting for farmer command.")
# #             # if state == RobotState.WAIT_FOR_COMMAND: stop relay in dynamic_obstacles.py

# #         elif self.state == RobotState.STOPPED:
# #             self.get_logger().info("Robot stopped. Awaiting further instructions.")
# #             # stop mirte_base_controller by stopping relay in dynamic_obstacles.py

# #         elif self.state == RobotState.NAV_TO_TREE:
# #             if self.nav_goal_reached:
# #                 self.nav_goal_reached = False  # reset flag
# #                 self.get_logger().info("Reached tree.")
# #                 self.state = RobotState.DETECTING_APPLES
# #             else:
# #                 self.get_logger().info("Navigating to tree...")
# #                 # Trigger navigation to tree
# #                 # publish goal location to nav2
# #                 # or use a service to set the goal
# #                 # For example, publish a new goal to the navigation stack
# #                 # or use a service to set the goal

# #         elif self.state == RobotState.DETECTING_APPLES:
# #             self.get_logger().info(f"Scanning for apples... ")

# #             if self.nr_of_apples_detected >= 1:
# #                 self.get_logger().info(f"Detected {self.nr_of_apples_detected} apples.")
# #                 self.nr_of_apples_detected = -1  # reset
# #                 self.state = RobotState.PICKING_APPLE
# #             elif self.nr_of_apples_detected == 0:
# #                 self.get_logger().info("🍏 No apple detected.")
# #                 self.nr_of_apples_detected = -1  # reset
# #                 self.state = RobotState.RETURNING_HOME

# #         elif self.state == RobotState.PICKING_APPLE:
# #             self.get_logger().info("Picking apple...")
# #             if self.gripper_above_base:
# #                 self.get_logger().info("Gripper full, navigating to basket.")
# #                 self.state = RobotState.NAV_TO_BASKET

# #         elif self.state == RobotState.NAV_TO_BASKET:
# #             if self.nav_goal_reached:
# #                 self.nav_goal_reached = False  # reset flag
# #                 self.get_logger().info("Reached basket.")
# #                 self.state = RobotState.PLACING_APPLE
# #             else:
# #                 self.get_logger().info("Navigating to basket...")
# #                 # Trigger navigation to basket
# #                 # publish goal location to nav2

# #         elif self.state == RobotState.PLACING_APPLE:
# #             self.get_logger().info("Placing apple in basket...")
# #             if not self.gripper_above_base:
# #                 self.get_logger().info("Apple placed. Returning to process order state")
# #                 self.state = RobotState.PROCESS_ORDER

# #         elif self.state == RobotState.OBSTACLE_AVOIDANCE:
# #             if not self.obstacle:
# #                 self.get_logger().info(
# #                     f"Obstacle cleared, returning to {self.previous_state}"
# #                 )
# #                 self.state = self.previous_state
# #             else:
# #                 self.get_logger().info("Avoiding obstacle...")

# #         elif self.state == RobotState.RETURNING_HOME:
# #             if self.obstacle:
# #                 self.state = RobotState.OBSTACLE_AVOIDANCE
# #                 self.get_logger().info(
# #                     "Obstacle detected, switching to avoidance mode."
# #                 )
# #             elif self.nav_goal_reached:
# #                 self.nav_goal_reached = False
# #                 self.get_logger().info("Reached home.")
# #                 self.previous_state = RobotState.RETURNING_HOME
# #                 # self.state = RobotState.COMMUNICATING #After it has returned to home, we don't want it to go back in the field automatically.
# #                 self.state = RobotState.WAIT_FOR_COMMAND
# #             else:
# #                 self.get_logger().info("Returning to start...")
# #                 # Trigger navigation to home
# #                 # publish goal location to nav2

# #         elif self.state == RobotState.COMMUNICATING:
# #             self.get_logger().info("Communicating status to farmer...")
# #             self.get_logger().info(
# #                 f"State to communicate to farmer: {self.previous_state.name}"
# #             )

# #             if self.previous_state == RobotState.RETURNING_HOME:
# #                 self.get_logger().info("All apples picked. Returning home.")
# #                 self.state = RobotState.RETURNING_HOME
# #             else:
# #                 self.state = RobotState.PROCESS_ORDER

# #         elif self.state == RobotState.EXCEPTION:
# #             self.get_logger().info("Handling exception...")
# #             # TODO: error recovery or escalate
# #             self.previous_state = RobotState.EXCEPTION
# #             self.state = RobotState.COMMUNICATING

# #         else:
# #             self.get_logger().warn("Unknown state, resetting to WAIT_FOR_COMMAND.")
# #             self.state = RobotState.WAIT_FOR_COMMAND


# # def main(args=None):
# #     rclpy.init(args=args)
# #     node = FSMNode()
# #     rclpy.spin(node)
# #     node.destroy_node()
# #     rclpy.shutdown()
