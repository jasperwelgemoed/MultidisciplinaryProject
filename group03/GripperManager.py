#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PointStamped
from std_msgs.msg import Bool, String
import numpy as np
from scipy.optimize import minimize
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory, GripperCommand
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import tf2_ros
import tf2_geometry_msgs
from visualization_msgs.msg import Marker
import time


class IKSolver(Node):
    def __init__(self):
        super().__init__("gripper_manager_node")

        # --- TF Buffer and Listener ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Declare ROS parameters for arm link lengths and initial configuration
        self.declare_parameters(
            "",
            [
                ("l1", 0.0281),
                ("l2", 0.1378),
                ("l3", 0.14265),
                ("l4", 0.0975),
                ("initial_q", [0.0, 0.0, 0.0, 0.0]),
                (
                    "approach_offset",
                    0.03,
                ),  # Offset for pre-grasp (e.g., approach 3cm above)
                ("unpluck_distance", 0.02),  # Up/down distance for unplucking
                ("unpluck_cycles", 2),  # Number of unpluck cycles
                (
                    "gripper_open_delay_sec",
                    2.0,
                ),  # Delay after dropping before returning to stow
            ],
        )

        # Get parameter values
        self.l1 = self.get_parameter("l1").value
        self.l2 = self.get_parameter("l2").value
        self.l3 = self.get_parameter("l3").value
        self.l4 = self.get_parameter("l4").value
        self.initial_q = np.array(self.get_parameter("initial_q").value)
        self.approach_offset = self.get_parameter("approach_offset").value
        self.unpluck_distance = self.get_parameter("unpluck_distance").value
        self.unpluck_cycles = self.get_parameter("unpluck_cycles").value
        self.gripper_open_delay_sec = self.get_parameter("gripper_open_delay_sec").value

        # --- Gripper Command Values ---
        self.gripper_closed_position = 0.4  # Value to close the gripper
        self.gripper_open_position = -0.1  # Value to open the gripper

        # --- Action Clients ---
        self.arm_client = ActionClient(
            self,
            FollowJointTrajectory,
            "/mirte_master_arm_controller/follow_joint_trajectory",
        )
        self.arm_joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_joint",
        ]

        self.gripper_client = ActionClient(
            self, GripperCommand, "/mirte_master_gripper_controller/gripper_cmd"
        )
        self.gripper_closed = False  # Internal state of gripper

        # --- Publishers ---
        self.gripper_above_base_pub = self.create_publisher(
            Bool, "/gripper_above_base", 10
        )
        self.marker_pub = self.create_publisher(
            Marker, "/apple_marker", 10
        )  # For visualization in RViz

        # --- Subscribers ---
        self.create_subscription(
            PointStamped,
            "/detections/gripper/apples/coords",
            self.apple_coords_callback,
            10,
        )
        # Visual servoing callback method remains as placeholder, but no active subscription.
        # self.create_subscription(Float32MultiArray, '/apple_tracking/error_uv',
        #                          self.visual_servo_callback, 10)

        self.create_subscription(Pose, "/arm_goal_pose", self.manual_pose_callback, 10)
        # Subscribe to FSM's current state (String) - This is the primary trigger for arm actions
        self.create_subscription(String, "/fsm_state", self.state_callback, 10)

        # --- State Variables ---
        self.is_busy = False  # True when arm is executing an *internal* complex task (picking, dropping sub-stages)
        self.stage = "idle"  # Internal stage of the arm's current sequence (e.g., "pre_grasp", "unplucking_up")
        self.fsm_current_state = (
            "WAIT_FOR_COMMAND"  # Stores the latest FSM state received
        )

        self.stored_apple_target = (
            None  # Stores the latest apple coordinates received from vision
        )
        # Visual servoing flags are now effectively unused since visual servoing is removed.
        self.visual_servo_active = False
        self.current_visual_servo_iteration = 0
        self.unplucking_current_cycle = 0

        # Predefined joint positions (radians)
        self.stow_position = np.array([0.0, 0.802851, -0.523599, 1.5708])
        self.drop_position = np.array(
            [np.deg2rad(0), np.deg2rad(-26), np.deg2rad(-85), np.deg2rad(-14)]
        )
        self.safe_return_position = np.array([0.0, 0.0, 1.56, 1.5708])

        # --- Timers and Checks ---
        self.arm_server_ready = False
        self.create_timer(
            1.0, self.check_arm_server
        )  # Periodically check if arm action server is ready
        self.gripper_open_timer = None  # Timer for delay after dropping apple

        # --- Initialization Publishers ---
        self.publish_gripper_above_base(True)

        self.get_logger().info("IK solver node started")

    # --- Kinematics Functions (class methods) ---
    def dh_matrix(self, theta, d, a, alpha):
        """
        Computes the Denavit-Hartenberg transformation matrix.
        """
        cos_theta = np.cos(theta)
        sin_theta = np.sin(theta)
        cos_alpha = np.cos(alpha)
        sin_alpha = np.sin(alpha)

        return np.array(
            [
                [
                    cos_theta,
                    -sin_theta * cos_alpha,
                    sin_theta * sin_alpha,
                    a * cos_theta,
                ],
                [
                    sin_theta,
                    cos_theta * cos_alpha,
                    -cos_theta * sin_alpha,
                    a * sin_theta,
                ],
                [0, sin_alpha, cos_alpha, d],
                [0, 0, 0, 1],
            ]
        )

    def forward_kinematics(self, q):
        """
        Computes the forward kinematics for the 4-DOF arm using instance's link lengths.
        """
        theta1 = q[0]
        A1 = self.dh_matrix(theta1, self.l1, 0.00625, np.pi / 2)

        theta2 = q[1] + np.pi / 2
        A2 = self.dh_matrix(theta2, 0, self.l2, 0)

        theta3 = q[2]
        A3 = self.dh_matrix(theta3, -0.00014, self.l3, 0)

        theta4 = q[3]
        A4 = self.dh_matrix(theta4, 0, self.l4, 0)

        T0_4 = A1 @ A2 @ A3 @ A4
        return T0_4[0:3, 3]

    def inverse_kinematics(self, target_pos, initial_q, max_iter=1000, tol=1e-8):
        """
        Solves the inverse kinematics problem using optimization.
        Uses instance's link lengths.
        """
        bounds = [
            (-np.deg2rad(90), np.deg2rad(90)),  # shoulder_pan_joint
            (-np.deg2rad(90), np.deg2rad(90)),  # shoulder_lift_joint
            (-np.deg2rad(90), np.deg2rad(90)),  # elbow_joint
            (-np.deg2rad(90), np.deg2rad(90)),  # wrist_joint
        ]

        min_safe_x = 0.15

        def objective(q):
            """Objective function to minimize the distance to the target position."""
            current_pos = self.forward_kinematics(q)
            error_norm = np.linalg.norm(current_pos - target_pos)

            penalty = 0.0
            if current_pos[0] > min_safe_x:
                penalty += 100.0 * (current_pos[0] - min_safe_x) ** 2

            return error_norm + penalty

        result = minimize(
            objective,
            initial_q,
            method="SLSQP",
            bounds=bounds,
            options={"maxiter": max_iter, "ftol": tol},
        )

        if result.success:
            return np.mod(result.x + np.pi, 2 * np.pi) - np.pi
        else:
            raise RuntimeError("IK failed: " + result.message)

    def transform_point(self, point_stamped, target_frame):
        """
        Transforms a PointStamped message to a target frame using the instance's TF buffer.
        """
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                point_stamped.header.frame_id,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0),
            )
            return tf2_geometry_msgs.do_transform_point(point_stamped, transform)
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            self.get_logger().error(f"Transform error: {e}")
            return None

    def check_arm_server(self):
        """Checks if the arm action server is available."""
        if not self.arm_server_ready and self.arm_client.server_is_ready():
            self.arm_server_ready = True
            self.get_logger().info("Arm action server is available!")
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn("Gripper action server not available yet.")

    def publish_gripper_above_base(self, state: bool):
        """Publishes the /gripper_above_base status."""
        msg = Bool()
        msg.data = state
        self.gripper_above_base_pub.publish(msg)
        self.get_logger().info(f"Publishing /gripper_above_base: {state}")

    def send_arm_trajectory(
        self, joint_positions: np.array, time_from_start_sec: float = 3.0
    ):
        """
        Sends a FollowJointTrajectory goal to the arm controller.
        """
        if not self.arm_server_ready:
            self.get_logger().warn("Arm server not ready, cannot send trajectory.")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)
            return

        goal_msg = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = self.arm_joint_names

        point = JointTrajectoryPoint()
        point.positions = joint_positions.tolist()
        point.velocities = [0.0] * len(joint_positions)
        point.accelerations = [0.0] * len(joint_positions)
        point.time_from_start.sec = int(time_from_start_sec)
        point.time_from_start.nanosec = int(
            (time_from_start_sec - int(time_from_start_sec)) * 1e9
        )

        trajectory.points = [point]
        goal_msg.trajectory = trajectory

        self.get_logger().info(
            f"Sending arm trajectory (Internal Stage: {self.stage}): {joint_positions}"
        )
        future = self.arm_client.send_goal_async(goal_msg)
        future.add_done_callback(self.arm_goal_response_callback)

    def arm_goal_response_callback(self, future):
        """Callback for the arm action server's goal acceptance/rejection."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error(f"Arm goal rejected! Internal Stage: {self.stage}")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)
            return
        self.get_logger().info(f"Arm goal accepted (Internal Stage: {self.stage})")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.handle_arm_result)

    def grip(self, position: float, max_effort: float = 0.0):
        """
        Sends a GripperCommand goal to the gripper controller.
        """
        if not self.gripper_client.server_is_ready():
            self.get_logger().warn(
                "Gripper action server not available, skipping grip command."
            )
            return

        goal_msg = GripperCommand.Goal()
        goal_msg.command.position = position
        goal_msg.command.max_effort = max_effort

        self.get_logger().info(f"Sending gripper goal: {position}")
        self.gripper_client.send_goal_async(goal_msg)
        self.gripper_closed = position > 0.0

    def apple_coords_callback(self, msg: PointStamped):
        """
        Callback for new apple coordinates from vision node. Stores them for later use when FSM state allows.
        """
        self.stored_apple_target = msg
        self.get_logger().info(
            f"Received and stored apple at {msg.point.x:.2f}, {msg.point.y:.2f}, {msg.point.z:.2f} (from vision)."
        )
        self.publish_apple_marker(np.array([msg.point.x, msg.point.y, msg.point.z]))

    def visual_servo_callback(self, msg):
        """
        Visual servoing logic is removed from the active FSM flow. This callback is left
        as a placeholder or if you later decide to re-introduce visual servoing.
        It will not be triggered unless a subscription is re-added for it.
        """
        self.get_logger().warn(
            "Visual servo callback received data but visual servoing is currently disabled in arm_control node."
        )
        pass

    def manual_pose_callback(self, msg: Pose):
        """
        Callback for manual arm goal pose.
        """
        if not self.arm_server_ready:
            self.get_logger().warn("Arm server not ready for manual pose.")
            return

        if self.is_busy:
            self.get_logger().info(
                f"Arm busy ({self.stage}), ignoring manual pose command."
            )
            return

        target = np.array([msg.position.x, msg.position.y, msg.position.z])
        self.get_logger().info(f"Manual target received: {target}")
        try:
            self.is_busy = True
            self.stage = "manual_control"
            self.publish_gripper_above_base(False)
            q = self.inverse_kinematics(target, self.initial_q)
            self.send_arm_trajectory(q)
            self.initial_q = q
        except Exception as e:
            self.get_logger().error(f"Manual IK error: {e}")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)

    def state_callback(self, msg: String):
        """
        Callback for FSM state updates. This is the primary trigger for arm actions.
        """
        self.fsm_current_state = msg.data
        self.get_logger().debug(f"Received FSM state: '{self.fsm_current_state}'")

        # If arm is already busy with an internal sequence, ignore new FSM commands for now.
        # It will complete its current task and then check FSM state again implicitly when idle.
        if self.is_busy:
            self.get_logger().debug(
                f"Arm is busy with internal stage: {self.stage}. Ignoring new FSM command '{self.fsm_current_state}' for now."
            )
            return

        # 1. Handle picking process initiation
        # Arm begins pick sequence when FSM state is PICKING_APPLE AND an apple target is available.
        if self.fsm_current_state == "PICKING_APPLE":  # Use exact FSM state name
            if self.stored_apple_target is not None:
                self.get_logger().info(
                    "FSM in PICKING_APPLE and apple target available. Initiating pick sequence."
                )

                # Transform apple point to base_link frame
                transformed_point_stamped = self.transform_point(
                    self.stored_apple_target, "base_link"
                )
                if transformed_point_stamped is None:
                    self.get_logger().error(
                        "Transform failed for stored apple coordinates, cannot initiate pick."
                    )
                    self.stored_apple_target = None
                    return

                apple_pos_base_link = np.array(
                    [
                        transformed_point_stamped.point.x,
                        transformed_point_stamped.point.y,
                        transformed_point_stamped.point.z,
                    ]
                )

                # Safety check for forward extension
                if apple_pos_base_link[0] > 0.15:
                    self.get_logger().warn(
                        f"Apple x={apple_pos_base_link[0]:.2f} is too far forward. Adjusting."
                    )
                    apple_pos_base_link[0] = 0.15

                try:
                    q_initial_approach = self.inverse_kinematics(
                        apple_pos_base_link, self.initial_q
                    )

                    self.initial_q = q_initial_approach
                    self.is_busy = True  # Set busy flag
                    self.publish_gripper_above_base(
                        False
                    )  # Arm is moving away from base
                    self.stage = "initial_approach_to_apple"  # Arm's internal pick sequence starts
                    self.send_arm_trajectory(q_initial_approach)
                    self.get_logger().info(
                        f"Started pick for apple at {apple_pos_base_link}. Internal Stage: {self.stage}"
                    )
                    # Do NOT clear stored_apple_target here. It is consumed by being acted upon by this new command.

                except Exception as e:
                    self.get_logger().error(
                        f"IK error initiating pick from FSM state: {e}. Clearing target."
                    )
                    self.is_busy = False
                    self.stage = "idle"
                    self.publish_gripper_above_base(True)
                    # self.stored_apple_target = None # Optionally clear on IK failure
            else:
                self.get_logger().debug(
                    "FSM in PICKING_APPLE, but no apple target stored yet. Waiting for vision to publish."
                )

        # 2. Handle placing process initiation (FSM in PLACING_APPLE)
        elif self.fsm_current_state == "PLACING_APPLE":  # Use exact FSM state name
            # Arm should be in 'waiting_for_basket' internal stage after a pick, before FSM commands drop.
            if self.stage == "waiting_for_basket":
                self.get_logger().info(
                    "FSM in PLACING_APPLE. Initiating drop sequence."
                )
                self.is_busy = True  # Set busy flag
                self.publish_gripper_above_base(False)  # Arm is moving away from base
                self.stage = "dropping_apple"  # Arm's internal drop sequence starts
                self.send_arm_trajectory(self.drop_position)
            else:
                self.get_logger().debug(
                    f"FSM in PLACING_APPLE, but arm not ready for drop (Internal Stage: {self.stage})."
                )
                # This could happen if FSM transitions faster than arm completes "stow_to_base" or is in a different internal stage.

        # 3. "Kill function" / default behavior for other FSM states
        # If FSM is in any state not related to arm operations, and arm is not busy with its internal stages,
        # ensure it's in a safe/idle state.
        else:  # FSM state is not PICKING_APPLE or PLACING_APPLE
            if (
                self.stage != "idle"
                and self.stage != "waiting_for_basket"
                and self.stage != "manual_control"
            ):
                self.get_logger().debug(
                    f"FSM state is {self.fsm_current_state}. Arm not busy. Ensuring idle state."
                )
                self.stage = "idle"
                self.publish_gripper_above_base(True)

    def handle_arm_result(self, future):
        """
        Callback executed when an arm trajectory goal completes.
        Manages the internal state transitions of the apple picking/dropping process.
        """
        status = future.result().status

        if status != 4:  # Not succeeded
            self.get_logger().error(
                f"Arm motion failed in internal stage: {self.stage} with status: {status}"
            )
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)
            return

        self.get_logger().info(
            f"Arm reached target in internal stage: {self.stage}. Transitioning to next step."
        )

        if self.stage == "initial_approach_to_apple":
            self.get_logger().info(
                "Reached initial apple position. Proceeding to pre-grasp."
            )
            self.stage = "pre_grasp"
            self.execute_pre_grasp()

        elif self.stage == "pre_grasp":
            self.get_logger().info("Pre-grasp position reached. Attempting to grasp.")
            self.stage = "grasping"
            self.grip(self.gripper_closed_position)
            self.get_logger().info("Gripper command sent. Assuming gripper closed.")

            self.stage = "unplucking_up"
            self.unplucking_current_cycle = 0
            self.get_logger().info(f"Transitioning to {self.stage}")
            self.execute_unplucking_cycle()

        elif self.stage in ["unplucking_up", "unplucking_down"]:
            self.unplucking_current_cycle += 0.5

            if self.unplucking_current_cycle < self.unpluck_cycles * 2:
                self.execute_unplucking_cycle()
            else:
                self.get_logger().info("Unplucking complete. Moving to stow position.")
                self.stage = "stow_to_base"
                self.send_arm_trajectory(self.stow_position)

        elif self.stage == "stow_to_base":
            self.get_logger().info(
                "Arm is stowed above base. Awaiting FSM's PLACING_APPLE or other command."
            )
            self.publish_gripper_above_base(
                True
            )  # Crucial: Signal FSM that arm is safe for base movement
            self.is_busy = False  # Arm is no longer busy with a pick sequence, ready for next FSM command
            self.stage = "waiting_for_basket"  # Internal state while waiting for FSM to command drop or next mission

        elif self.stage == "dropping_apple":
            self.get_logger().info("Reached drop position. Opening gripper.")
            self.grip(self.gripper_open_position)
            self.gripper_closed = False
            self.stage = "releasing_apple"
            self.gripper_open_timer = self.create_timer(
                self.gripper_open_delay_sec, self.after_apple_release_delay
            )

        elif self.stage == "return_to_stow":
            self.get_logger().info(
                "Arm returned to stow position after drop. Resetting."
            )
            self.publish_gripper_above_base(True)
            self.is_busy = False
            self.stored_apple_target = None  # Clear any lingering apple target
            self.stage = "idle"

        elif self.stage == "manual_control":
            self.get_logger().info("Manual control trajectory completed.")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)

    def execute_pre_grasp(self):
        """
        Calculates and sends the command to move the gripper very close to the apple,
        ready for grasping. This is triggered by visual servoing convergence.
        """
        if self.stored_apple_target is None:
            self.get_logger().error("No stored apple target for pre-grasp. Aborting.")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)
            return

        transformed_point_stamped = self.transform_point(
            self.stored_apple_target, "base_link"
        )
        if transformed_point_stamped is None:
            self.get_logger().error(
                "Transform failed for stored apple coords during pre-grasp, aborting."
            )
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)
            self.stored_apple_target = None
            return

        pre_grasp_target_pos = np.array(
            [
                transformed_point_stamped.point.x,
                transformed_point_stamped.point.y,
                transformed_point_stamped.point.z,
            ]
        )

        pre_grasp_target_pos[0] += self.approach_offset

        self.get_logger().info(f"Moving to pre-grasp position: {pre_grasp_target_pos}")
        try:
            q_pre_grasp = self.inverse_kinematics(pre_grasp_target_pos, self.initial_q)
            self.initial_q = q_pre_grasp
            self.stage = "pre_grasp"
            self.send_arm_trajectory(q_pre_grasp, time_from_start_sec=1.5)
        except Exception as e:
            self.get_logger().error(f"IK error for pre-grasp: {e}. Aborting picking.")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)

    def execute_unplucking_cycle(self):
        """Executes one up or down movement for unplucking."""
        if self.stored_apple_target is None:
            self.get_logger().error("No stored apple target for unplucking. Aborting.")
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)
            return

        current_effector_pos = self.forward_kinematics(self.initial_q)
        unpluck_target_pos = current_effector_pos.copy()

        if (self.unplucking_current_cycle * 2) % 2 == 0:
            unpluck_target_pos[2] += self.unpluck_distance
            self.stage = "unplucking_up"
            self.get_logger().info(
                f"Executing unpluck cycle {int(self.unplucking_current_cycle)}: Moving up {self.unpluck_distance}m."
            )
        else:
            unpluck_target_pos[2] -= self.unpluck_distance
            self.stage = "unplucking_down"
            self.get_logger().info(
                f"Executing unpluck cycle {int(self.unplucking_current_cycle)}: Moving down {self.unpluck_distance}m."
            )

        try:
            q_unpluck = self.inverse_kinematics(unpluck_target_pos, self.initial_q)
            self.initial_q = q_unpluck
            self.send_arm_trajectory(q_unpluck, time_from_start_sec=1.0)
        except Exception as e:
            self.get_logger().error(
                f"IK error during unplucking: {e}. Aborting picking."
            )
            self.is_busy = False
            self.stage = "idle"
            self.publish_gripper_above_base(True)

    def after_apple_release_delay(self):
        """Callback for the timer after opening gripper to allow apple to drop."""
        self.get_logger().info(
            f"Delay after apple release complete. Moving to return to stow."
        )
        if self.gripper_open_timer:
            self.gripper_open_timer.cancel()
            self.destroy_timer(self.gripper_open_timer)
            self.gripper_open_timer = None

        self.stage = "return_to_stow"
        self.send_arm_trajectory(self.safe_return_position)

    def publish_apple_marker(self, position: np.array):
        """Publishes a marker for visualization in RViz."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "apple_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = position[0]
        marker.pose.position.y = position[1]
        marker.pose.position.z = position[2]
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.7
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = IKSolver()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
