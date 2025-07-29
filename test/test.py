import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Header
import numpy as np
from scipy.optimize import minimize


def dh_matrix(theta, d, a, alpha):
    cos_theta = np.cos(theta)
    sin_theta = np.sin(theta)
    cos_alpha = np.cos(alpha)
    sin_alpha = np.sin(alpha)

    return np.array(
        [
            [cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, a * cos_theta],
            [sin_theta, cos_theta * cos_alpha, -cos_theta * sin_alpha, a * sin_theta],
            [0, sin_alpha, cos_alpha, d],
            [0, 0, 0, 1],
        ]
    )


def forward_kinematics(q, l1, l2, l3, l4):
    theta1 = q[0]
    A1 = dh_matrix(theta1, l1, 0.00625, np.pi / 2)
    theta2 = q[1] + np.pi / 2
    A2 = dh_matrix(theta2, 0, l2, 0)
    theta3 = q[2]
    A3 = dh_matrix(theta3, -0.00014, l3, 0)
    theta4 = q[3]
    A4 = dh_matrix(theta4, 0, l4, 0)
    T = A1 @ A2 @ A3 @ A4
    return T[0:3, 3]


def inverse_kinematics(target_pos, initial_q, l1, l2, l3, l4, max_iter=1000, tol=1e-8):
    bounds = [(-np.pi / 2, np.pi / 2)] * 4

    def objective(q):
        return np.linalg.norm(forward_kinematics(q, l1, l2, l3, l4) - target_pos)

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


class IKAndTrajectoryNode(Node):
    def __init__(self):
        super().__init__("ik_trajectory_combined_node")

        # Parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("l1", 0.0281),
                ("l2", 0.1378),
                ("l3", 0.14265),
                ("l4", 0.0975),
                ("initial_q", [0.0, 0.0, 0.0, 0.0]),
            ],
        )
        self.l1 = self.get_parameter("l1").value
        self.l2 = self.get_parameter("l2").value
        self.l3 = self.get_parameter("l3").value
        self.l4 = self.get_parameter("l4").value
        self.initial_q = np.array(self.get_parameter("initial_q").value)

        # Publishers and Subscriber
        self.subscription = self.create_subscription(
            Point, "/arm_goal_point", self.pose_callback, 10
        )
        self.joint_pub = self.create_publisher(JointState, "/ik_solution", 10)
        self.trajectory_pub = self.create_publisher(
            JointTrajectory, "/mirte_master_arm_controller/joint_trajectory", 10
        )

        self.get_logger().info("IK and Trajectory node initialized")

    def pose_callback(self, msg):
        target_pos = np.array([msg.x, msg.y, msg.z])
        try:
            q = inverse_kinematics(
                target_pos, self.initial_q, self.l1, self.l2, self.l3, self.l4
            )
            reached_pos = forward_kinematics(q, self.l1, self.l2, self.l3, self.l4)

            if np.linalg.norm(reached_pos - target_pos) <= 0.1:
                # JointState message (for monitoring/debugging)
                joint_msg = JointState()
                joint_msg.header.stamp = self.get_clock().now().to_msg()
                joint_msg.name = [
                    "shoulder_pan_joint",
                    "shoulder_lift_joint",
                    "elbow_joint",
                    "wrist_joint",
                ]
                joint_msg.position = q.tolist()
                self.joint_pub.publish(joint_msg)

                # JointTrajectory message (for control)
                traj = JointTrajectory()
                traj.header.stamp = self.get_clock().now().to_msg()
                traj.joint_names = joint_msg.name
                point = JointTrajectoryPoint()
                point.positions = q.tolist()
                point.time_from_start.sec = 3
                traj.points.append(point)
                self.trajectory_pub.publish(traj)

                self.get_logger().info(f"Solution: {q}, Position: {reached_pos}")
                self.initial_q = q  # Update guess
            else:
                self.publish_error()
                self.get_logger().warn(
                    f"Position error too high: {np.linalg.norm(reached_pos - target_pos):.3f}"
                )
        except Exception as e:
            self.get_logger().error(str(e))
            self.publish_error()

    def publish_error(self):
        error_msg = JointState()
        error_msg.header.stamp = self.get_clock().now().to_msg()
        error_msg.name = ["joint1", "joint2", "joint3", "joint4"]
        error_msg.position = [-100.0] * 4
        self.joint_pub.publish(error_msg)


def main(args=None):
    rclpy.init(args=args)
    node = IKAndTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
