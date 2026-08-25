#!/usr/bin/env python3
# encoding: utf-8
#物体位姿计算
import os
import cv2
import yaml
import time
import rclpy
import numpy as np
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image as RosImage, CameraInfo
 

class CoordinateTransformation(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.K = None
        self.white_area_center = None
        self.camera_type = os.environ['CAMERA_TYPE']
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/"


        # 订阅图像话题
        self.image_sub = self.create_subscription(RosImage, '/color_detection/result_image', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)

    def get_yaml(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])

        tvec = extristric[:1]  # 取第一行
        rmat = extristric[1:]  # 取后面三行

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.03)
        self.extristric = tvec, rmat

    def camera_info_callback(self, msg):
        self.K = np.matrix(msg.k).reshape(1, -1, 3)

    # 处理ROS节点数据
    def image_callback(self, result_image):
        try:
            
            # 将 ROS Image 消息转换为 OpenCV 图像
            result_image = self.bridge.imgmsg_to_cv2(result_image, "mono8")

            if result_image is not None :
                # 计算识别到的轮廓
                contours = cv2.findContours(result_image, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓

                if contours :

                    # 找出最大轮廓
                    c = max(contours, key = cv2.contourArea)
                    # 根据轮廓大小判断是否进行下一步处理
                    rect = cv2.minAreaRect(c)  # 获取最小外接矩形
                    corners = np.int0(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
                    x, y = rect[0][0],rect[0][1]
                    if self.camera_type == 'USB_CAM':
                        x, y = distortion_inverse_map.undistorted_to_distorted_pixel(target[2][0], target[2][1], self.intrinsic, self.distortion)

                    self.get_yaml()
                    projection_matrix = np.row_stack((np.column_stack((self.extristric[1],self.extristric[0])), np.array([[0, 0, 0, 1]])))
                    world_pose = common.pixels_to_world([[x,y]], self.K, projection_matrix)[0]
                    world_pose[0] = -world_pose[0]
                    world_pose[1] = -world_pose[1]
                    position = self.white_area_center[:3, 3] + world_pose
                    position[2] = 0.03
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['pixel']['offset'])
                    scale = tuple(config_data['pixel']['scale'])
                    for i in range(3):
                        position[i] = position[i] * scale[i]
                        position[i] = position[i] + offset[i]

                    # 打印像素坐标、实际坐标
                    self.get_logger().info(f"像素坐标为: x: {x}, y: {y}")
                    self.get_logger().info(f"实际坐标为： {position}")
                    time.sleep(1)
                else:
                    self.get_logger().info("未检测到所需识别的颜色，请将色块放置到相机视野内。")

        except Exception as e:
            print(e)

def main():
    node = CoordinateTransformation('coordinate_transformation')
    rclpy.spin(node)
    camera_node.destroy_node()
    
    rclpy.shutdown()
        
if __name__ == '__main__':
    main()
