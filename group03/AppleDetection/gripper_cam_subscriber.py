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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

from std_msgs.msg import String
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point

import cv2
from cv_bridge import CvBridge

# from ultralytics import YOLO  # Make sure to load the correct YOLO model
import numpy as np

# TODO:
# publish if there is an apple in the picker
# if apple in gripper, update gripper full


class GripperCamSubscriber(Node):

    def __init__(self):
        super().__init__("gripper_cam_subscriber")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.subscription = self.create_subscription(
            CompressedImage,
            "/gripper_camera/image_raw/compressed",  # gripper cam
            self.apple_grip_detection,
            qos_profile,
        )
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()
        self.i = 0

    def apple_grip_detection(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)  # decode from compressed format

        cv2.imshow(f"frame", frame)
        cv2.waitKey(1)

        self.get_logger().info(f"I see frame {self.i}")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    gripper_cam_subscriber = GripperCamSubscriber()

    rclpy.spin(gripper_cam_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    gripper_cam_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
