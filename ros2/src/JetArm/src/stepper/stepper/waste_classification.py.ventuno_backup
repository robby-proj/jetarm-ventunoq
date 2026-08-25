#!/usr/bin/env python3
# encoding: utf-8
# 垃圾分类

import os
import cv2
import yaml
import time
import copy
import math
import queue
import rclpy
import signal
import threading
import numpy as np
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from stepper import stepper as Stepper
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetStringList
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position
from app.utils import calculate_grasp_yaw, position_change_detect, pick_and_place, image_process, distortion_inverse_map

TARGET_POSITION = {
    'recyclable_waste': (5200),
    'hazardous_waste': (4250),
    'food_waste': (3300),
    'residual_waste': (2300)
}

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}


class WasteClassificationNode(Node):


    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.start_get_roi = True
        self.last_position = None
        self.roi = None
        self.imgpts = None
        self.count_move = 0
        self.count_still = 0
        self.last_position = None
        self.extristric = None
        self.intrinsic = None
        self.distortion = None
        self.target = None
        self.start_transport = False
        self.target_object_info = None
        self.stepper = Stepper.Stepper(7)
        self.stepper.set_mode(self.stepper.EN) # 设置滑轨使能
        self.camera_type = os.environ['CAMERA_TYPE']
        self.calibration_file = 'calibration.yaml'
        self.config_file = 'transform.yaml'
        self.config_path = "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/stepper/config/"
        self.target_list =['BananaPeel', 'BrokenBones', 'Ketchup', 'Marker', 'OralLiquidBottle', 'StorageBattery', 'PlasticBottle', 'Toothbrush', 'Umbrella', 'Plate', 'CigaretteEnd', 'DisposableChopsticks']

        self.bridge = CvBridge()
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        self.image_sub = self.create_subscription(Image, '/yolov8/object_image', self.image_callback, 1)
        self.object_sub = self.create_subscription(ObjectsInfo, '/yolov8/object_detect', self.get_object_callback, 1)

        #等待服务启动
        self.timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish', callback_group=self.timer_cb_group)
        self.client.wait_for_service()
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose', callback_group=self.timer_cb_group)
        self.get_current_pose_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=self.timer_cb_group)
        self.kinematics_client.wait_for_service()
        self.start_yolov8_client = self.create_client(Trigger, '/yolov8/start', callback_group=self.timer_cb_group)
        self.start_yolov8_client.wait_for_service()
        self.stop_yolov8_client = self.create_client(Trigger, '/yolov8/stop', callback_group=self.timer_cb_group)
        self.stop_yolov8_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)


    def init_process(self):
        self.timer.cancel()
        set_servo_position(self.joints_pub, 1.0, ( (1, 500),(2, 520), (3, 210), (4, 50), (5, 500), (10, 200)))  # 设置机械臂初始位置
        time.sleep(1)
        self.stepper.go_home()
        self.send_request(self.start_yolov8_client, Trigger.Request())
        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        cv2.drawContours(bgr_image, [self.imgpts], -1, (0, 255, 255), 2, cv2.LINE_AA) # 绘制矩形(draw rectangle)
        cv2.imshow('image', bgr_image)
        cv2.waitKey(1)

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            corners = np.array(config['corners']).reshape(-1, 3)
            self.white_area_center = np.array(config['white_area_pose_world'])
        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)

        tvec = extristric[:1]  # 取第一行
        rmat = extristric[1:]  # 取后面三行

        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.04)
        self.extristric = tvec, rmat
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.0)
        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic, distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)
        self.imgpts = imgpts

        # 裁切出ROI区域(crop RIO region)
        x_min = min(imgpts, key=lambda p: p[0])[0] # x轴最小值(the minimum value of X-axis)
        x_max = max(imgpts, key=lambda p: p[0])[0] # x轴最大值(the maximum value of X-axis)
        y_min = min(imgpts, key=lambda p: p[1])[1] # y轴最小值(the minimum value of Y-axis)
        y_max = max(imgpts, key=lambda p: p[1])[1] # y轴最大值(the maximum value of Y-axis)
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)
            
        self.roi = roi

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.03):
        projection_matrix = np.row_stack((np.column_stack((extristric[1], extristric[0])), np.array([[0, 0, 0, 1]])))
        world_pose = common.pixels_to_world([position], intrinsic, projection_matrix)[0]
        world_pose[0] = -world_pose[0]
        world_pose[1] = -world_pose[1]
        position = white_area_center[:3, 3] + world_pose
        position[2] = height
        
        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
        offset = tuple(config_data['pixel']['offset'])
        scale = tuple(config_data['pixel']['scale'])
        for i in range(3):
            position[i] = position[i] * scale[i]
            position[i] = position[i] + offset[i]
        return position, projection_matrix

    def calculate_pick_grasp_yaw(self, position, target, target_info, intrinsic, projection_matrix):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        # 0.09x0.02
        gripper_size = [common.calculate_pixel_length(0.09, intrinsic, projection_matrix),
                        common.calculate_pixel_length(0.02, intrinsic, projection_matrix)]

        return calculate_grasp_yaw.calculate_gripper_yaw_angle(target, target_info, gripper_size, yaw)

    def calculate_place_grasp_yaw(self, position, angle=0):
        yaw = math.degrees(math.atan2(position[1], position[0]))
        if position[0] < 0 and position[1] < 0:
            yaw = yaw + 180
        elif position[0] < 0 and position[1] > 0:
            yaw = yaw - 180
        yaw1 = yaw + angle
        if yaw < 0:
            yaw2 = yaw1 + 90
        else:
            yaw2 = yaw1 - 90

        yaw = yaw2
        if abs(yaw1) < abs(yaw2):
            yaw = yaw1
        yaw = 500 + int(yaw / 240 * 1000)

        return yaw

    def transport_thread(self):
        while True:
            if self.start_transport:
                self.send_request(self.stop_yolov8_client, Trigger.Request())
                position, yaw, target = self.transport_info
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                for i in range(3):
                    position[i] = position[i] * scale[i]
                    position[i] = position[i] + offset[i]
                # self.get_logger().info(f'pick2:{position}')

                finish = pick_and_place.pick(position, 80, yaw, 500, 0.02, self.joints_pub, self.kinematics_client)
                if finish:
                    set_servo_position(self.joints_pub, 1.0, ( (1, 500),))  
                    time.sleep(1.0)
                    stepper_position = TARGET_POSITION[target]
                    self.stepper.goto(stepper_position) # 驱动滑轨移动到放置位置
                    stepper_time = stepper_position/1000 # 计算需要的时间
                    time.sleep(stepper_time)

                    position =  [0.25, 0, 0.02] #设置放置位置

                    yaw = self.calculate_place_grasp_yaw(position, 0)
                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['kinematics']['offset'])
                    scale = tuple(config_data['kinematics']['scale'])
                    angle = math.degrees(math.atan2(position[1], position[0]))
                    if angle > 45:
                        # self.get_logger().info(f'1:{position}')
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] - offset[1], position[1] + offset[0], position[2] + offset[2]]
                    elif angle < -45:
                        # self.get_logger().info(f'2:{position}')
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
                    else:
                        # self.get_logger().info(f'3:{position}')
                        position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
                        position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]

                    # self.get_logger().info(f'{position}')
                    finish = pick_and_place.place(position, 80, yaw, 200, self.joints_pub, self.kinematics_client)
                    if finish:
                        set_servo_position(self.joints_pub, 1.5, ( (1, 500),(2, 520), (3, 210), (4, 50), (5, 500), (10, 200)))  # 设置机械臂初始位置
                        time.sleep(1.5)
                        stepper_position = TARGET_POSITION[target]
                        self.stepper.goto(-stepper_position) # 驱动滑轨移动到放置位置
                        stepper_time = stepper_position/1000 # 计算需要的时间
                        time.sleep(stepper_time)
                        self.send_request(self.start_yolov8_client, Trigger.Request())
                        self.target = None
                        self.start_transport = False
            else:
                time.sleep(0.1)

    def main(self):
        while self.running:
            if self.start_get_roi:
                self.get_roi()
                self.start_get_roi = False
            roi = self.roi
            if self.target_object_info is not None:
                target_object_info = copy.deepcopy(self.target_object_info)
                center = target_object_info[0][2]
                if self.camera_type == 'USB_CAM':
                    x, y = distortion_inverse_map.undistorted_to_distorted_pixel(center[0], center[1], self.intrinsic, self.distortion)
                    center = (x, y)
                intrinsic = self.intrinsic
                self.target_object_info = None

                if roi[2] < center[0] < roi[3] and roi[0] < center[1] < roi[1]:
                    position, projection_matrix = self.get_object_world_position(target_object_info[0][2], intrinsic, self.extristric, self.white_area_center, 0.04)

                    result = self.calculate_pick_grasp_yaw(position, target_object_info[0], target_object_info[1], intrinsic, projection_matrix)
                    if result is not None:
                        if self.last_position is not None:
                            e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(pow(self.last_position[1] - position[1], 2)), 5)
                            # self.get_logger().info(f'{e_distance}')
                            if e_distance <= 0.002:  # 欧式距离小于2mm, 防止物体还在移动时就去夹取了
                                self.count_move = 0
                                self.count_still += 1
                            else:
                                self.count_move += 1
                                self.count_still = 0
                            if self.count_move > 10:
                                if target_object_info[0][0] in self.target_list:
                                    self.target_list.remove(target_object_info[0][0])

                            if self.count_still > 10:
                                self.count_still = 0
                                self.count_move = 0
                                for k, v in WASTE_CLASSES.items():
                                    if target_object_info[0][0] in v:
                                        waste_category = k
                                        break
                                yaw = 500 + int(result[0] / 240 * 1000)
                                self.transport_info = [position, yaw, waste_category]
                                # self.get_logger().info(f'{self.transport_info}')
                                self.start_transport = True
                        self.last_position = position

    def get_object_callback(self, msg):
        objects = msg.objects
        local_target_object_info = None
        local_objects_list = []
        local_object_info = None
        class_name = None

        target_object_info = self.target_object_info
        if not self.start_transport and objects and target_object_info is None:
            for i in objects:
                # 计算对象的角度
                if i.angle < 0:
                    i.angle = 90 - abs(i.angle)

                target = [i.class_name, 0, (int(i.box[0]), int(i.box[1])), (int(i.box[2]), int(i.box[3])), i.angle]

                # 如果找到目标对象
                if i.class_name in self.target_list:
                    if local_object_info is None:
                        local_object_info = target

                    if local_object_info[0] == i.class_name:
                        class_name = i.class_name
                        local_object_info = target

                local_objects_list.append(target)

            if class_name is not None:
                local_target_object_info = [local_object_info, local_objects_list]
        if not self.start_transport and target_object_info is None:
            self.target_object_info = copy.deepcopy(local_target_object_info)


def main():
    node = WasteClassificationNode('waste_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()


