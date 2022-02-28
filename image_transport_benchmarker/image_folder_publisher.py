#!/usr/bin/env python
from __future__ import print_function

import sys
import os
from os import listdir
from os.path import isfile, join

import rclpy
from rclpy.node import Node
import cv2

from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

from std_msgs.msg import Bool
import time 

class image_folder_publisher(Node):
    def __init__(self):
        super().__init__("image_folder_publisher")
        self.__app_name = "image_folder_publisher"

        self._cv_bridge = CvBridge()

        self.total_imgs = 3000

        self._topic_name = '/camera/image_raw'
        self.get_logger().info(f"{self.__app_name} (topic_name) Publishing Images to topic {self._topic_name}")

        self._image_publisher = self.create_publisher(Image, self._topic_name, 10)

        self._rate = 30
        self.get_logger().info(f"{self.__app_name} (publish_rate) Publish rate set to {self._rate} hz")

        self.seq = 0

        self._image_folder = '/home/karthikdharmarajan/Documents/rgbd_dataset_freiburg1_xyz/rgb'
        if self._image_folder == '' or not os.path.exists(self._image_folder) or not os.path.isdir(self._image_folder):
            sys.exit(0)
        # print(f"{self.__app_name} Reading images from {self._image_folder}")
        self.files_in_dir = [f for f in listdir(self._image_folder) if isfile(join(self._image_folder, f))]
        self.img = 0

        self.stop_pub = self.create_publisher(Bool, '/stop', 1)        
        
        self.start_sub = self.create_subscription(Bool, '/start', self.start_images, 10)


    def stop_callback(self):
        self.get_logger().info("Sending stop!")
        self.stop_pub.publish(Bool())

    def start_images(self, msg):
        self.get_logger().info("Starting to send images!")
        self.timer = self.create_timer(1 / self._rate, self.run)

    def run(self):
        try:
            if self.total_imgs <= self.seq:
                time.sleep(30)
                self.stop_callback()
                return
            if len(self.files_in_dir) <= self.img:
                self.img = 0
            f = self.files_in_dir[self.img]
            if isfile(join(self._image_folder, f)):
                cv_image = cv2.imread(join(self._image_folder, f))
                if cv_image is not None:
                    ros_msg = self._cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
                    ros_msg.header.frame_id = str(self.seq)
                    ros_msg.header.stamp = self.get_clock().now().to_msg()
                    self._image_publisher.publish(ros_msg)
                    self.seq += 1
                    # print(f"{self.__app_name} Published {join(self._image_folder, f)}")
                else:
                    print(f"{self.__app_name} Invalid image file {join(self._image_folder, f)}")
                self.img += 1
        except CvBridgeError as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    image_publisher = image_folder_publisher()
    rclpy.spin(image_publisher)
    rclpy.shutdown()

if __name__ == '__main__':
    main(sys.argv)
