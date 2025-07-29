# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import tf2_ros
import tf2_geometry_msgs

from std_msgs.msg import String, Bool, Int32
from sensor_msgs.msg import Image, Range, CompressedImage, CameraInfo
from geometry_msgs.msg import Point, PoseArray, Pose
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

import cv2
from cv_bridge import CvBridge
from ultralytics import YOLO  # Make sure to load the correct YOLO model
import numpy as np


# BASE CAM INFO:
# frame_id mirte: camera_color_optical_frame
# frame_id gazebo: camera_rgb_frame
# TODO:
# use sonar to get Z (depth) coordinate of tree
# transform locations from camera frame to gripper frame?
# if no apples detected, move base back a bit
# of detected apples, get red ones (or remove green ones)
# ros2 run group03 talker --> publishes an example image for the listener to do the talking. not necessary if you're using mirte's camera feed
# ros2 run group03 base_cam --> activates processing of base cam (tree/apple detection)
# ros2 run group03 gripper_cam --> activates gripper cam processing


class BaseCamSubscriber(Node):

    def __init__(self):
        super().__init__("base_cam_subscriber")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        #### Subscribers
        self.base_camera_subscription = self.create_subscription(
            CompressedImage,
            "/camera/color/image_raw/compressed",
            self.detection,
            qos_profile,
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "/camera/color/camera_info", self.set_camera_info, 1
        )
        self.sonar_subscription = self.create_subscription(
            Range, "io/distance/front_left", self.set_range, 1
        )
        self.fsm_state_subscription = self.create_subscription(
            String, "fsm_state", self.fsm_callback, 1
        )

        ### Publishers
        # For FSM
        self.tree_detection_bool_publisher = self.create_publisher(
            Bool, "/detections/tree/bool", 10
        )
        # self.apple_detected_bool_publisher = self.create_publisher(Bool, 'apple_detected', 10)
        self.apple_detected_bool_publisher = self.create_publisher(
            Int32, "apple_detected", 10
        )
        # For base/arm path planning
        self.tree_detection_coord_publisher = self.create_publisher(
            Point, "/detections/tree/coord", 10
        )
        self.apple_detection_coord_publisher = self.create_publisher(
            PoseArray, "/detections/apples/coords", 10
        )
        # TODO:
        # publisher (int type) thats 0,1,2 none (or "NONE") by default.
        # once we've detected apples set to true, once no more apples activitely set to false.

        # Current state (variable)
        self.active = False
        self.i = 0
        self.tree_found = Bool()
        self.apple_detected = (
            0  # 0 is not started looking for apples, 1 is true, 2 is empty tree
        )
        self.area_of_interest = (-1, -1, -1, -1)
        self.depth = -1.0

        # Utils (persistent)
        self.intrinsic_matrix = None
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.model = YOLO("best2.pt")
        self.bridge = CvBridge()

    """
    Sets necessary info from CameraInfo message once.
    """

    def set_camera_info(self, msg):
        if self.intrinsic_matrix is None:
            self.intrinsic_matrix = msg.k

    """
    Project 2D image pixel coordinates + depth measurement into 3D camera coordinates.
    (ChatGPT)
    """

    def deproject_pixel_to_point(self, u, v, depth, K):
        fx = K[0]
        fy = K[4]
        cx = K[2]
        cy = K[5]

        X = (u - cx) * depth / fx
        Y = (v - cy) * depth / fy
        Z = depth
        return X, Y, Z

    """
    Set the global depth variable to the incoming depth measurement.
    """

    def set_range(self, msg):
        self.depth = msg.range

    """
    Set current FSM state.
    """

    def fsm_callback(self, msg):
        self.active = True if msg.data == "DETECTING_APPLES" else False

    """
    Finds trees by detecting largest green area of the camera image.

    Note: Partially derived using ChatGPT.
    """

    def tree_detection(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # decode from compressed format
        # frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        cv2.imshow(f"frame", frame)
        cv2.waitKey(1)
        hsv_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define hue threshold range to isolate green hue
        # Hue values in OpenCV range from 0 to 179
        lower_hue = 30
        upper_hue = 90

        # # Define lower and upper bounds for thresholding
        lower_bound = np.array([lower_hue, 50, 50])
        upper_bound = np.array([upper_hue, 255, 255])

        # Apply the threshold
        mask = cv2.inRange(hsv_frame, lower_bound, upper_bound)

        # Morphological operations
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.erode(mask, kernel, iterations=1)
        mask = cv2.dilate(mask, kernel, iterations=2)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Find the largest contour by area
            largest_contour = max(contours, key=cv2.contourArea)

            # Get bounding box
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Check if ratio is "correct" for a tree in full view (roughly 3:1 rectangle)
            # TODO: and that it is roughly in the center?
            if w <= 4 * h and w >= 2.5 * h:

                # Draw bounding box on original image
                boxed_image = frame.copy()
                cv2.rectangle(boxed_image, (x, y), (x + w, y + h), (0, 255, 0), 2)

                # Optionally draw contour
                cv2.drawContours(boxed_image, [largest_contour], -1, (255, 0, 0), 1)

                # Display results
                cv2.imshow("Bounding Box", boxed_image)
                cv2.waitKey(1)

                # Update current state
                self.area_of_interest = (x, y, w, h)
                self.tree_found.data = True
                self.get_logger().info("Tree detected")
                self.tree_detection_bool_publisher.publish(self.tree_found)

                # Publish location
                cx, cy = (x + x + w) // 2, (y + y + h) // 2
                point_in_cam_frame = Point()
                point_in_cam_frame.x, point_in_cam_frame.y, point_in_cam_frame.z = (
                    self.deproject_pixel_to_point(
                        cx, cy, self.depth, self.intrinsic_matrix
                    )
                )
                self.tree_detection_coord_publisher.publish(point_in_cam_frame)

                # Annotate image
                cv2.imshow("Tree", boxed_image)
                cv2.waitKey(1)

                return frame, (x, y, w, h)

            else:
                self.get_logger().info("No tree detected")
                self.tree_found.data = False
                self.tree_detection_bool_publisher.publish(self.tree_found)
                self.apple_found.data = 0
                return frame, (-1, -1, -1, -1)
        else:
            self.get_logger().info("No tree detected")
            self.tree_found.data = False
            self.tree_detection_bool_publisher.publish(self.tree_found)
            self.apple_found.data = 0
            return frame, (-1, -1, -1, -1)

    """
    Look within given area of interest for apples using YOLO classifier.

    Note: Partially derived using ChatGPT.
    """

    def get_apple_locations(self, frame, area_of_interest):
        x, y, w, h = area_of_interest

        # run YOLO to detect apples
        results = self.model(frame)[0]

        # initialize things for filling in during for loop
        apple_locations = PoseArray()
        apple_locations.header.frame_id = "camera_color_optical_frame"
        crop_imgs = []
        annotated_frame = frame.copy()

        # iterate through all the detections
        for det in results.boxes:
            class_id = int(det.cls)
            label = self.model.names[class_id]

            x1, y1, x2, y2 = map(int, det.xyxy[0])
            cx, cy = (x1 + x2) // 2, (y1 + y2) // 2

            # check if detection is within area of interest
            if x1 >= x and x2 <= x + w and y1 >= y and y2 <= y + h:

                if label == "apple_red":
                    # Publish location
                    point_in_cam_frame = Point()
                    point_in_cam_frame.x, point_in_cam_frame.y, point_in_cam_frame.z = (
                        self.deproject_pixel_to_point(
                            float(cx), float(cy), self.depth, self.intrinsic_matrix
                        )
                    )

                    pose = Pose()
                    pose.position = point_in_cam_frame
                    apple_locations.poses.append(pose)

                    # Annotate image
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 0, 255), 2)
                    cv2.putText(
                        annotated_frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                    )
                    cv2.putText(
                        annotated_frame,
                        f"{cx}, {cy}",
                        (x1, y2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 0, 255),
                        2,
                    )
                else:
                    # Annotate image
                    cv2.rectangle(annotated_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(
                        annotated_frame,
                        label,
                        (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )
                    cv2.putText(
                        annotated_frame,
                        f"{cx}, {cy}",
                        (x1, y2 + 15),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (0, 255, 0),
                        2,
                    )

                crop_img = frame[y1:y2, x1:x2]
                crop_imgs.append(crop_img)
                crop_msg = self.bridge.cv2_to_imgmsg(crop_img)

                self.get_logger().info(f"I found apple at: {cx}, {cy}")
                cv2.imshow("detections", annotated_frame)
                cv2.waitKey(1)

        if apple_locations:
            self.apple_detected.data = 1  # Apples found
            self.apple_detection_coord_publisher.publish(apple_locations)
        else:
            self.apple_detected.data = 2  # No apples found on tree

    """
    Main controller for this node. Determines what to run when.
    """

    def detection(self, msg):
        self.camera_frame = msg.header.frame_id

        if self.intrinsic_matrix is None:
            self.get_logger().warn("Skipping detection — data not yet received.")
            return

        # If actively in apple searching mode (from FSM)
        elif (
            self.active
        ):  # TODO: CHANGE THIS BACK. keep as "True" for testing purposes only
            frame, area_of_interest = self.tree_detection(msg)
            if self.tree_found:
                self.get_apple_locations(frame, area_of_interest)
                self.apple_detected_bool_publisher.publish(self.apple_detected)
            else:
                self.apple_detected = 0
                self.apple_detected_bool_publisher.publish(self.apple_detected)


def main(args=None):
    rclpy.init(args=args)

    base_cam_subscriber = BaseCamSubscriber()

    rclpy.spin(base_cam_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
