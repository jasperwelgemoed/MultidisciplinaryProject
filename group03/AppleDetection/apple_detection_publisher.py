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

from std_msgs.msg import String
from std_msgs.msg import Header
from sensor_msgs.msg import Image

import os
import cv2
from cv_bridge import CvBridge


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__("minimal_publisher")
        self.publisher_ = self.create_publisher(Image, "/camera/test/image_raw", 10)
        timer_period = 1.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.bridge = CvBridge()
        # self.image = cv2.imread('src/group03/group03/apple_detection/test_img.jpg')  # test image

        # if self.image is None:
        #     self.get_logger().error('Failed to load image.')
        # else:
        #     self.get_logger().info('Image loaded and publishing started.')

    def timer_callback(self):
        idx = str(int(self.i % 18))
        # make sure to run this from ~/mdp_ws
        img_path = os.path.join(
            "src", "group03", "group03", "apple_detection", "test_imgs", f"{idx}.jpg"
        )
        self.get_logger().info(f"Loading image from: {img_path}")
        image = cv2.imread(img_path)

        image = cv2.resize(image, (640, 480), interpolation=cv2.INTER_LINEAR)
        # cv2.imshow("a", image)
        # cv2.waitKey(1)

        # image = cv2.imread('/home/lserrano/mdp_sim/src/group03/group03/apple_detection/test_imgs/{idx}.jpg')  # test image
        if image is None:
            self.get_logger().error("Failed to load image.")
        else:
            self.get_logger().info("Image loaded and publishing started.")
        # msg = self.bridge.cv2_to_imgmsg(self.image, encoding="bgr8")
        msg = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")

        self.publisher_.publish(msg)
        self.get_logger().info("published image")
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
