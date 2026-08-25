#!/usr/bin/env python3
# encoding: utf-8
# 摄像头调用
import cv2
import rclpy
import queue
import threading
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

        self.image_queue = queue.Queue(maxsize=2)
        
        # 创建 CvBridge 对象，用于将 ROS Image 转为 OpenCV 格式
        self.bridge = CvBridge()

        # 注册 Ctrl+C 中断信号处理器
        signal.signal(signal.SIGINT, self.shutdown)

        # 图像订阅者
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        ohreading.Thread(target=self.image_processing, daemon=True).start()
    def image_processing(self):
        while self.running:
            self.image = self.image_queue.get()
            if self.running and self.image is not None:
            #展示图像
                cv2.imshow('image', self.image)
                cv2.waitKey(1)

    def shutdown(self, signum, frame):
        self.get_logger().info("Shutting down...")
        self.running = False
        rclpy.shutdown()

    # 处理 ROS Image 消息并转化为 OpenCV 格式
    def image_callback(self, ros_image):
        try:
            # 将 ROS Image 消息转换为 OpenCV 图像
            self.image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")

            if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
                self.image_queue.get()
        # 将图像放入队列
            self.image_queue.put(self.image)

            # if self.running and self.image is not None:
                # 展示图像
                # cv2.imshow('image', self.image)
                # cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")

   

def main():
    camera_node = Camera('camera_topic_invoke')
    rclpy.spin(camera_node)
    camera_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

