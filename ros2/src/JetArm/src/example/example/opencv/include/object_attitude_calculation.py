#!/usr/bin/env python3
# encoding: utf-8
#物体位姿计算
import cv2
import rclpy
from rclpy.node import Node
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ObjectAttitudeCalculation(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        # 订阅图像话题
        self.image_sub = self.create_subscription(Image, '/color_detection/result_image', self.image_callback, 1)
        
    # 处理ROS节点数据
    def image_callback(self, result_image):
        try:
            
            # 将 ROS Image 消息转换为 OpenCV 图像
            result_image = self.bridge.imgmsg_to_cv2(result_image, "mono8")

            # 将灰度图像转换为 BGR 图像
            # result_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2GRAY)

            
            if result_image is not None :
                # 计算识别到的轮廓
                contours = cv2.findContours(result_image, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓

                if contours :

                    # 找出最大轮廓
                    c = max(contours, key = cv2.contourArea)
                    # 根据轮廓大小判断是否进行下一步处理
                    rect = cv2.minAreaRect(c)  # 获取最小外接矩形
                    yaw = int(round(rect[2]))  # 矩形角度
                    # 打印物体姿态
                    self.get_logger().info(f" 物体姿态:,yaw: {yaw}")
                    
                else:
                    self.get_logger().info("请将需要识别颜色的物体放入摄像头的识别范围")

        except Exception as e:
            print(e)

def main():
    node = ObjectAttitudeCalculation('object_attitude_calculation')
    rclpy.spin(node)
    camera_node.destroy_node()
    
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
