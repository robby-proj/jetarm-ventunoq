#!/usr/bin/env python3
# encoding: utf-8
#肢体骨骼检测
"""
这个程序实现了人体骨架识别(this program implements human skeleton recognition)
运行现象：桌面显示识别结果画面， 显示人体骨架连线(runtime behavior: the desktop displays the recognition result screen, showing the human skeleton lines)
"""
import cv2
import rclpy
import queue
import threading
import numpy as np
import sdk.fps as fps
import mediapipe as mp
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position


class MankindPoseNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)

        # 实例化一个肢体识别器(instantiate a limb recognizer)
        self.pose = mp.solutions.pose.Pose(
            static_image_mode=False,
            model_complexity=0,
            min_detection_confidence=0.8,
            min_tracking_confidence=0.2
        )
        self.drawing = mp.solutions.drawing_utils # 结果绘制工具(result drawing tool)
        self.fps = fps.FPS() # 帧率计数器(frame rate calculator)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换

        self.image_queue = queue.Queue(maxsize=2)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        #等待服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        set_servo_position(self.joints_pub, 1.5, ((10, 500), (5, 500), (4, 330), (3, 100), (2, 700), (1, 500)))

        threading.Thread(target=self.main, daemon=True).start()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)

    def main(self):
        while True:
            bgr_image = self.image_queue.get()
            bgr_image = cv2.flip(bgr_image, 1)  # 镜像画面, 这样可以正对屏幕和相机看效果(mirror image, aligned with the screen and camera for better visualization)
            result_image = np.copy(bgr_image) # 将画面复制一份作为结果，结果绘制在这上面(duplicate the image as the result canvas, and draw the results on it)
            # bgr_image = cv2.resize(bgr_image, (int(ros_image.width / 2), int(ros_image.height / 2))) 
            results = self.pose.process(bgr_image)  # 进行识别(perform recognition)
            self.drawing.draw_landmarks(result_image, results.pose_landmarks, mp.solutions.pose.POSE_CONNECTIONS) # 画出各关节及连线(draw the joints and lines connecting them)
            self.fps.update() # 计算帧率(calculate frame rate)
            self.fps.show_fps(result_image)
            cv2.imshow("result", result_image)
            cv2.waitKey(1)

        cv2.destroyAllWindows()
        rclpy.shutdown()


def main():
    node = MankindPoseNode('mankind_pose')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
    

