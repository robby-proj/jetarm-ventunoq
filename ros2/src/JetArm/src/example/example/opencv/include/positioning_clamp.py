#!/usr/bin/env python3
# encoding: utf-8
#定位夹取
import os
import cv2
import yaml
import time
import math
import rclpy
import threading
import numpy as np
from sdk import common
from math import radians
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from servo_controller import bus_servo_control
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState
from sensor_msgs.msg import Image as RosImage, CameraInfo
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller_msgs.msg import ServosPosition, ServoPosition


class PositioningClamp(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.K = None
        self.count = 0
        self.pick_pitch = 80
        self.result_image = None
        self.camera_type = os.environ['CAMERA_TYPE']
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/"

        self.previous_pose = None  # 上一次检测到的位置
        self.start = True

        self.servos_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)# 舵机控制

        #订阅图像话题
        self.image_sub = self.create_subscription(RosImage, '/color_detection/result_image', self.image_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)


        #等待服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        threading.Thread(target=self.run, daemon=True).start()

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
        # 将 ROS Image 消息转换为 OpenCV 图像
        self.result_image = self.bridge.imgmsg_to_cv2(result_image, "mono8")

            
    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
    
    def start_sorting(self, pose_t, pose_R):
        self.get_logger().info("开始搬运堆叠...")
        msg = set_pose_target(pose_t, self.pick_pitch,  [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse : # 可以达到
            servo_data = res.pulse  
            self.get_logger().info(f"舵机角度: {list(res.pulse)}")
          
            # 驱动舵机
            bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))
            time.sleep(2)
            
            bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((10, 550),))
            time.sleep(2)

            bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, 500), (2, 520), (3, 210), (4, 50), (5, 550) ))  # 设置机械臂初始位置
            time.sleep(2)

        else:
            self.start = False
            self.get_logger().info("无法运行到此位置")


    def run(self):
        while True:
            try:
                if self.result_image is not None :
                    # 计算识别到的轮廓
                    contours = cv2.findContours(self.result_image, cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓

                    if contours :
                        # 找出最大轮廓
                        c = max(contours, key = cv2.contourArea)
                        # 根据轮廓大小判断是否进行下一步处理
                        rect = cv2.minAreaRect(c)  # 获取最小外接矩形
                        corners = np.intp(cv2.boxPoints(rect))  # 获取最小外接矩形的四个角点
                        x, y, yaw = rect[0][0],rect[0][1],rect[2]
                        if self.camera_type == 'USB_CAM':
                            x, y = distortion_inverse_map.undistorted_to_distorted_pixel(target[2][0], target[2][1], self.intrinsic, self.distortion)

                        self.get_yaml()
                        projection_matrix = np.row_stack((np.column_stack((self.extristric[1],self.extristric[0])), np.array([[0, 0, 0, 1]])))
                        world_pose = common.pixels_to_world([[x,y]], self.K, projection_matrix)[0]
                        world_pose[0] = -world_pose[0]
                        world_pose[1] = -world_pose[1]
                        position = self.white_area_center[:3, 3] + world_pose
                        world_pose[2] = 0.03
                        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                        offset = tuple(config_data['pixel']['offset'])
                        scale = tuple(config_data['pixel']['scale'])
                        for i in range(3):
                            position[i] = position[i] * scale[i]
                            position[i] = position[i] + offset[i]
                        position[2] = 0.015
                         # 如果previous_pose为None，说明是第一次计算
                        if self.previous_pose is None:
                            self.previous_pose = position
                            continue

                        # 计算当前位置与上次位置的差异
                        position_difference = np.linalg.norm(np.array(position) - np.array(self.previous_pose))

                        # 判断位置变化是否足够小
                        if position_difference < 0.01:  # 可以根据需要调整阈值
                            self.count += 1  # 如果位置变化小，计数器+1

                        else:
                            self.count = 0  # 如果位置变化较大，计数器重置
                            self.previous_pose = position
                        
                        if self.count >= 60:  # 如果计数器达到阈值，则认为位置已稳定
                            config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                            offset = tuple(config_data['kinematics']['offset'])
                            scale = tuple(config_data['kinematics']['scale'])
                            for i in range(3):
                                position[i] = position[i] * scale[i]
                                position[i] = position[i] + offset[i]
                            # 打印像素坐标和实际坐标
                            self.get_logger().info(f"像素坐标为: x: {x}, y: {y}")
                            self.get_logger().info(f"实际坐标为： {position}")
                            self.start_sorting(position,yaw)
                            self.count = 0
                            break
                    else:
                        self.get_logger().info('未检测到所需识别的颜色，请将色块放置到相机视野内。')

            except Exception as e:
                self.get_logger().error(f"发生错误: {e}")

def main():
    node = PositioningClamp('positioning_clamp')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == '__main__':
    main()

