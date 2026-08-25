#!/usr/bin/env python3
# encoding: utf-8
# 垃圾分类
import os
import cv2
import yaml
import time
import math
import copy
import queue
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from sdk import common
from interfaces.msg import ObjectsInfo
from interfaces.srv import SetStringList
from kinematics_msgs.srv import SetRobotPose
from servo_controller_msgs.msg import ServosPosition
from servo_controller.bus_servo_control import set_servo_position
from app.utils import calculate_grasp_yaw, pick_and_place, distortion_inverse_map

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}

class WasteClassificationNode(Node):
    place_position = {'residual_waste': [0.087, -0.225, 0.02],
                      'food_waste': [0.025, -0.225, 0.02],
                      'hazardous_waste': [-0.036, -0.225, 0.02],
                      'recyclable_waste': [-0.098, -0.225, 0.02]}

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self._init_parameters()
        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/"
        self.camera_type = os.environ['CAMERA_TYPE']
        
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)

        self.timer_cb_group = ReentrantCallbackGroup()
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_srv = self.create_service(SetBool, '~/enable_transport', self.start_srv_callback)
        self.create_service(SetStringList, '~/set_target', self.set_target_srv_callback)

        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        self.start_yolov8_client = self.create_client(Trigger, 'yolov8/start', callback_group=self.timer_cb_group)
        self.start_yolov8_client.wait_for_service()
        self.stop_yolov8_client = self.create_client(Trigger, 'yolov8/stop', callback_group=self.timer_cb_group)
        self.stop_yolov8_client.wait_for_service()

        #等待服务启动
        self.client = self.create_client(Trigger, 'controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, 'kinematics/init_finish')
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target', callback_group=self.timer_cb_group)
        self.kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _init_parameters(self):
        self.heart = None
        self.target_list_temp = []
        self.target_list = []
        self.start_transport = False
        self.enable_transport = False
        self.waste_category = None
        self.count_move = 0
        self.count_still = 0
        self.count_miss = 0
        self.last_position = None
        self.start_get_roi = False
        self.target_object_info = None
        self.intrinsic = None
        self.distortion = None
        self.extristric = None
        self.white_area_center = None
        self.roi = None
        self.enter = False
        self.image_sub = None
        self.object_sub = None
        self.camera_info_sub = None   
        self.display = self.get_parameter('display').value
        self.app = self.get_parameter('app').value

    def init_process(self):
        self.timer.cancel()

        threading.Thread(target=self.main, daemon=True).start()
        threading.Thread(target=self.transport_thread, daemon=True).start()
        if self.get_parameter('start').value:
            self.enter_srv_callback(Trigger.Request(), Trigger.Response())
            req = SetBool.Request()
            req.data = True 
            res = SetBool.Response()
            self.start_srv_callback(req, res)

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def go_home(self, interrupt=True):
        if self.waste_category == "recyclable_waste":
            t = 2.0
        elif self.waste_category == "hazardous_waste":
            t = 1.7
        elif self.waste_category == "food_waste":
            t = 1.4
        elif self.waste_category == "residual_waste":
            t = 1.0
        else :
            t = 1.0
        if interrupt:
            set_servo_position(self.joints_pub, 0.5, ((10, 200),))
            time.sleep(0.5)

        set_servo_position(self.joints_pub, 1.0, ((2, 520), (3, 210), (4, 50), (5, 500)))
        time.sleep(1.0)
        set_servo_position(self.joints_pub, t, ((1, 500),))
        time.sleep(t)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def get_roi(self):
        with open(self.config_path + self.config_file, 'r') as f:
            config = common.get_yaml_data(os.path.join(self.config_path, self.config_file))
            # 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            self.white_area_center = np.array(config['white_area_pose_world'])
            corners = np.array(config['corners']).reshape(-1, 3)

        while True:
            intrinsic = self.intrinsic
            distortion = self.distortion
            if intrinsic is not None and distortion is not None:
                break
            time.sleep(0.1)
        
        tvec = extristric[:1]  # 取第一行
        rmat = extristric[1:]  # 取后面三行
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.04)
        extristric = tvec, rmat
        self.extristric = extristric
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.05)
        imgpts, jac = cv2.projectPoints(corners[:-1], np.array(rmat), np.array(tvec), intrinsic,
                                     distortion)
        imgpts = np.int32(imgpts).reshape(-1, 2)

        # 裁切出ROI区域(crop RIO region)
        x_min = min(imgpts, key=lambda p: p[0])[0]  # x轴最小值(the minimum value of X-axis)
        x_max = max(imgpts, key=lambda p: p[0])[0]  # x轴最大值(the maximum value of X-axis)
        y_min = min(imgpts, key=lambda p: p[1])[1]  # y轴最小值(the minimum value of Y-axis)
        y_max = max(imgpts, key=lambda p: p[1])[1]  # y轴最大值(the maximum value of Y-axis)
        roi = np.maximum(np.array([y_min, y_max, x_min, x_max]), 0)

        self.roi = roi

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter waste classification")
        set_servo_position(self.joints_pub, 1.0, ((1, 500), (2, 520), (3, 210), (4, 50), (5, 500), (10, 200)))
        self._init_parameters()
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        self.camera_info_sub = self.create_subscription(CameraInfo, 'depth_cam/rgb/camera_info', self.camera_info_callback, 1)
        self.image_sub = self.create_subscription(Image, 'yolov8/object_image', self.image_callback, 1)
        self.object_sub = self.create_subscription(ObjectsInfo, 'yolov8/object_detect', self.get_object_callback, 1)
        self.enter = True
        self.start_get_roi = True
        
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            self.get_logger().info('\033[1;32m%s\033[0m' % "exit waste classification")
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.destroy_subscription(self.object_sub)
                self.destroy_subscription(self.camera_info_sub)
                self.image_sub = None
                self.object_sub = None
                self.camera_info_sub = None
            self.send_request(self.stop_yolov8_client, Trigger.Request())
            self.heart.destroy()
            self.heart = None
            pick_and_place.interrupt(True)
            self.enter = False
            self.start_transport = False
        # self.get_logger().info('\033[1;32m%s\033[0m' % "exit waste classification")
        response.success = True
        response.message = "exit"
        return response

    def start_srv_callback(self, request, response):
        if request.data:
            if self.app:
                target_list = []
                target_list.extend(list(WASTE_CLASSES['food_waste']))
                target_list.extend(list(WASTE_CLASSES['hazardous_waste']))
                target_list.extend(list(WASTE_CLASSES['recyclable_waste']))
                target_list.extend(list(WASTE_CLASSES['residual_waste']))
                self.target_list = target_list
                self.target_list_temp = copy.deepcopy(self.target_list)
            self.send_request(self.start_yolov8_client, Trigger.Request())
            self.get_logger().info('\033[1;32m%s\033[0m' % "start waste classification")
            pick_and_place.interrupt(False)
            self.enable_transport = True
            response.message = "start"
        else:
            self.send_request(self.stop_yolov8_client, Trigger.Request())
            self.get_logger().info('\033[1;32m%s\033[0m' % "stop waste classification")
            pick_and_place.interrupt(True)
            self.enable_transport = False
            response.message = "stop"
        response.success = True
        return response

    def set_target_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set target " + str(request.data))
        target_list = []
        for i in request.data:
            target_list.extend(list(WASTE_CLASSES[i]))
            
        self.target_list = target_list
        self.target_list_temp = copy.deepcopy(self.target_list)
        self.get_logger().info('\033[1;32m%s\033[0m' % str(self.target_list))

        response.success = True
        response.message = "set target"
        return response

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                self.send_request(self.stop_yolov8_client, Trigger.Request())
                position, yaw, target = self.transport_info
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                for i in range(3):
                    position[i] = position[i] * scale[i]
                    position[i] = position[i] + offset[i]
                self.get_logger().info(f'pick2:{position}')
                finish = pick_and_place.pick(position, 80, yaw, 470, 0.02, self.joints_pub, self.kinematics_client)
                if finish:
                    position = copy.deepcopy(self.place_position[target])

                    yaw = self.calculate_place_grasp_yaw(position, 0)

                    config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                    offset = tuple(config_data['kinematics']['offset'])
                    scale = tuple(config_data['kinematics']['scale'])
                    angle = math.degrees(math.atan2(position[1], position[0]))
                    if angle > 45:
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] - offset[1], position[1] + offset[0], position[2] + offset[2]]
                    elif angle < -45:
                        position = [position[0] * scale[1], position[1] * scale[0], position[2] * scale[2]]
                        position = [position[0] + offset[1], position[1] - offset[0], position[2] + offset[2]]
                    else:
                        position = [position[0] * scale[0], position[1] * scale[1], position[2] * scale[2]]
                        position = [position[0] + offset[0], position[1] + offset[1], position[2] + offset[2]]
                    finish = pick_and_place.place(position, 80, yaw, 200, self.joints_pub, self.kinematics_client)
                    if finish:
                        self.go_home(False)
                    else:
                        self.go_home(True)
                else:
                    self.go_home(True)
                if self.enter:
                    if self.app:
                        self.target_list = copy.deepcopy(self.target_list_temp)
                    self.waste_category = None
                    self.start_transport = False
                    if self.enable_transport:
                        self.send_request(self.start_yolov8_client, Trigger.Request())
            else:
                time.sleep(0.1)

    def get_object_world_position(self, position, intrinsic, extristric, white_area_center, height=0.04):
        projection_matrix = np.row_stack(
            (np.column_stack((extristric[1], extristric[0])), np.array([[0, 0, 0, 1]])))
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

    def main(self):
        while self.running:
            if self.enter:
                if self.start_get_roi:
                    self.get_roi()
                    self.start_get_roi = False
                roi = self.roi
                if roi is not None:
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
                                    e_distance = round(
                                        math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                            pow(self.last_position[1] - position[1], 2)), 5)
                                    # self.get_logger().info(f'{e_distance}')
                                    if e_distance <= 0.005:  # 欧式距离小于2mm, 防止物体还在移动时就去夹取了
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
                                        self.waste_category = waste_category
                                        yaw = 500 + int(result[0] / 240 * 1000)
                                        # self.get_logger().info(f'{position}')
                                        self.transport_info = [position, yaw, waste_category]
                                        self.start_transport = True
                                self.last_position = position
                        else:
                            if target_object_info[0][0] in self.target_list:
                                self.target_list.remove(target_object_info[0][0])
                        if not self.target_list:
                            self.target_list = copy.deepcopy(self.target_list_temp)

                    else:
                        if self.enable_transport:
                            self.count_miss += 1
                            if self.count_miss > 10:
                                self.target_list = copy.deepcopy(self.target_list_temp)
                                self.count_miss = 0
                        time.sleep(0.02)
                else:
                    time.sleep(0.02)
            else:
                time.sleep(0.1)

    def get_object_callback(self, msg):
        objects = msg.objects

        local_target_object_info = None
        local_objects_list = []
        local_object_info = None
        class_name = None
            
        enable_transport = self.enable_transport
        target_object_info = self.target_object_info
        if enable_transport and objects and target_object_info is None:
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
        if enable_transport and target_object_info is None:
            self.target_object_info = copy.deepcopy(local_target_object_info)

    def camera_info_callback(self, msg):
        self.intrinsic = np.matrix(msg.k).reshape(1, -1, 3)
        self.distortion = np.array(msg.d)

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.display:
            cv2.imshow('image', bgr_image)
            cv2.waitKey(1)
        else:
            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))

def main():
    node = WasteClassificationNode('waste_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False  # 停止线程标志
        executor.shutdown()

if __name__ == "__main__":
    main()
