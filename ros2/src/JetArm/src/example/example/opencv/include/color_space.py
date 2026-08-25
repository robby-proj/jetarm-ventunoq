#!/usr/bin/env python3
# encoding: utf-8
#颜色空间转换
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import signal

class Camera(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.image = None
        self.running = True
        self.image_sub = None

        
        # 创建 CvBridge 对象，用于将 ROS Image 转为 OpenCV 格式
        self.bridge = CvBridge()

    
        # 图像订阅者
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        

    def shutdown(self, signum, frame):
        self.get_logger().info("Shutting down...")
        self.running = False
        rclpy.shutdown()

    # 处理 ROS Image 消息并转化为 OpenCV 格式
    def image_callback(self, ros_image):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            self.rgb_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")

            self.bgr_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2BGR)

            # 将图像的颜色空间转换成LAB
            self.lab_image = cv2.cvtColor(self.rgb_image, cv2.COLOR_RGB2LAB)
            
            if self.running  and self.bgr_image is not None and self.lab_image is not None:
                # 展示图像
                cv2.imshow('rgb_image', self.rgb_image)
                cv2.imshow('bgr_image', self.bgr_image)
                cv2.imshow('lab_image', self.lab_image)
                cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

   

def main():
    camera_node = Camera('color_space')
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

