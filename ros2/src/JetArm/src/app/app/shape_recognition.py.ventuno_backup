#!/usr/bin/python3
# coding=utf8
# 通过深度图识别物体进行分类
# 机械臂向下识别
# 可以识别长方体，球，圆柱体，以及他们的颜色
import os
import cv2
import yaml
import time
import math
import queue
import threading
import numpy as np

import rclpy
import message_filters
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from tf2_ros import Buffer, TransformListener, TransformException

from sdk import common, fps
from app.common import Heart
from interfaces.srv import SetStringList
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from servo_controller.bus_servo_control import set_servo_position
from servo_controller.action_group_controller import ActionGroupController
from kinematics_msgs.srv import SetJointValue, SetRobotPose, SetLink, GetLink
from kinematics.kinematics_control import set_joint_value_target, set_pose_target
from app.utils import utils, calculate_grasp_yaw_by_depth, position_change_detect, pick_and_place

class ShapeRecognitionNode(Node):
    hand2cam_tf_matrix = [
        [0.0, 0.0, 1.0, -0.101],  # x
        [-1.0, 0.0, 0.0, 0.01],  # y
        [0.0, -1.0, 0.0, 0.05],  # z
        [0.0, 0.0, 0.0, 1.0]
    ]
    '''
                
                 x1(+)
        y1(+)   center    y2(-)
                  x2

                  arm
    '''

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.fps = fps.FPS()
        self.running = True
        self._init_parameters()
        self.lock = threading.RLock()
        self.gripper_depth = 0.015
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.image_queue = queue.Queue(maxsize=2)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)

        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.enable_srv = self.create_service(SetBool, '~/set_running', self.start_srv_callback)
        self.rgb_or_depth_srv = self.create_service(SetBool, '~/rgb_or_depth', self.rgb_or_depth_srv_callback)
        self.client = self.create_client(Trigger, 'controller_manager/init_finish')
        self.create_service(SetStringList, '~/set_shape', self.set_shape_srv_callback)
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        
        self.client = self.create_client(Trigger, 'controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(SetBool, 'depth_cam/set_ldp_enable')
        self.client.wait_for_service()
       
        self.timer_cb_group = ReentrantCallbackGroup()
        self.set_joint_value_target_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target', callback_group=self.timer_cb_group)
        self.set_joint_value_target_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()
        self.get_link_client = self.create_client(GetLink, 'kinematics/get_link')
        self.get_link_client.wait_for_service()
        self.set_link_client = self.create_client(SetLink, 'kinematics/set_link')
        self.set_link_client.wait_for_service()

        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/factory_utils/arm_pc/ActionGroups')

        self.config_file = 'transform.yaml'
        self.calibration_file = 'calibration.yaml'
        self.config_path = "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/"
        self.base_gripper_height = utils.get_gripper_size(500)[1]
        tf_buffer = Buffer()
        self.tf_listener = TransformListener(tf_buffer, self)
        tf_future = tf_buffer.wait_for_transform_async(
            target_frame='depth_cam_depth_optical_frame',
            source_frame='depth_cam_color_frame',
            time=rclpy.time.Time()
        )

        rclpy.spin_until_future_complete(self, tf_future)
        try:
            transform = tf_buffer.lookup_transform(
                'depth_cam_color_frame', 'depth_cam_depth_frame', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=5.0))
            self.static_transform = transform  # 保存变换数据
            # self.get_logger().info(f'Static transform: {self.static_transform}')
        except TransformException as e:
            self.get_logger().error(f'Failed to get static transform: {e}')

        # 提取平移和旋转
        translation = transform.transform.translation
        rotation = transform.transform.rotation

        transform_matrix = common.xyz_quat_to_mat([translation.x, translation.y, translation.z],
                                                  [rotation.w, rotation.x, rotation.y, rotation.z])
        self.hand2cam_tf_matrix = np.matmul(transform_matrix, self.hand2cam_tf_matrix)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)          

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
            self.shapes = ['cuboid', 'sphere', 'cylinder']

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def _init_parameters(self):
        self.heart = None
        self.sync = None
        self.start_transport = False
        self.enter = False
        self.count_still = 0
        self.count_move = 0
        self.enable_transport = False
        self.plane = []
        self.display_rgb = True
        self.shapes = None
        self.target = None
        self.image_sub = None
        self.depth_sub = None
        self.info_sub = None
        self.depth_info_sub = None
        self.target_shape = None
        self.endpoint = None
        self.corners = []
        self.extristric = []
        self.last_position = 0, 0
        self.last_object_info_list = []
        self.display = self.get_parameter('display').value
        self.app = self.get_parameter('app').value

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def go_home(self, interrupt=True):
        if interrupt:
            set_servo_position(self.joints_pub, 0.5, ((10, 200), ))
            time.sleep(0.5)
        
        joint_angle = [500, 520, 210, 50, 500]
        
        msg = set_joint_value_target(joint_angle)
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t, pose_r = endpoint.pose.position, endpoint.pose.orientation
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        set_servo_position(self.joints_pub, 1, ((2, joint_angle[1]), (3, joint_angle[2]), (4, joint_angle[3]), (5, 500)))
        time.sleep(1)

        set_servo_position(self.joints_pub, 1, ((1, joint_angle[0]), (10, 200)))
        time.sleep(1)

    def enter_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "enter shape recognition")
        self._init_parameters()
        self.heart = Heart(self, '~/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        config = common.get_yaml_data(os.path.join(self.config_path, self.config_file))
        self.plane = config['plane']
        self.corners = np.array(config['corners'])
        self.extristric = np.array(config['extristric'])

        if self.sync is None:
            self.rgb_sub = message_filters.Subscriber(self, Image, 'depth_cam/rgb/image_raw')
            self.depth_sub = message_filters.Subscriber(self, Image, 'depth_cam/depth/image_raw')
            self.depth_info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/depth/camera_info')
            self.info_sub = message_filters.Subscriber(self, CameraInfo, 'depth_cam/rgb/camera_info')

            # 同步时间戳, 时间允许有误差在0.03s
            self.sync = message_filters.ApproximateTimeSynchronizer([self.rgb_sub, self.depth_sub, self.info_sub, self.depth_info_sub], 3, 0.2)
            self.sync.registerCallback(self.multi_callback)
        self.enter = True
        joint_angle = [500, 520, 210, 50, 500]

        set_servo_position(self.joints_pub, 1, ((1, joint_angle[0]), (2, joint_angle[1]), (3, joint_angle[2]), (4, joint_angle[3]), (5, 500), (10, 200)))
        msg = set_joint_value_target(joint_angle)
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t, pose_r = endpoint.pose.position, endpoint.pose.orientation
        self.endpoint = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        response.success = True
        response.message = "start"
        return response

    def exit_srv_callback(self, request, response):
        if self.enter:
            self.get_logger().info('\033[1;32m%s\033[0m' % "exit shape recognition")
            if self.sync is not None:
                self.sync = None
                self.rgb_sub = None
                self.depth_sub = None
                self.depth_info_sub = None
                self.info_sub = None
            self.heart.destroy()
            self.heart = None
            pick_and_place.interrupt(True)
            self.enter = False
            self.start_transport = False
        response.success = True
        response.message = "stop"
        return response

    def start_srv_callback(self, request, response):
        if request.data:
            self.get_logger().info('\033[1;32m%s\033[0m' % "start shape recognition")
            if self.app:
                msg = SetStringList.Request()
                msg.data = ['sphere', 'cuboid', 'cylinder']
                self.set_shape_srv_callback(msg, SetStringList.Response())
            pick_and_place.interrupt(False)
            self.enable_transport = True
            response.success = True
            response.message = "start"
            return response
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % "stop shape recognition")
            self.enable_transport = False
            self.target_shape = None
            pick_and_place.interrupt(True)
            response.success = False
            response.message = "stop"
            return response

    def set_shape_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "set_shape")
        self.shapes = request.data
        self.get_logger().info('\033[1;32m%s\033[0m' % str(self.shapes))

        response.success = True
        response.message = "set_shape"
        return response

    def rgb_or_depth_srv_callback(self, request, response):
        self.display_rgb = request.data
        response.success = True
        return response

    def transport_thread(self):
        while self.running:
            if self.start_transport:
                msg = BuzzerState()
                msg.freq = 1900
                msg.on_time = 0.2
                msg.off_time = 0.01
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                time.sleep(1)
                
                shape, position, yaw = self.transport_info[0], self.transport_info[2], self.transport_info[-1]
                if position[0] > 0.22:
                    position[2] += 0.01
                config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                offset = tuple(config_data['kinematics']['offset'])
                scale = tuple(config_data['kinematics']['scale'])
                for i in range(3):
                    position[i] = position[i] * scale[i]
                    position[i] = position[i] + offset[i]

                # self.get_logger().info(f"shape: {shape}, position: {position}, yaw: {yaw}")
                if "sphere" in shape:
                    finish = pick_and_place.pick(position, 85, yaw, 570, 0.05, self.joints_pub, self.kinematics_client)
                else:
                    finish = pick_and_place.pick(position, 85, yaw, 570, 0.02, self.joints_pub, self.kinematics_client)
                if finish:
                    # self.get_logger().info(f'111111111:{shape}')
                    if "sphere" in shape:
                        self.controller.run_action("target_1")
                    if "cylinder" in shape:
                        self.controller.run_action("target_2")
                    if "cuboid" in shape:
                        self.controller.run_action("target_3")
                    self.go_home(False)
                else:
                    self.go_home(True)
                self.start_transport = False
            else:
                time.sleep(0.1)

    def shape_recognition(self, bgr_image, depth_image, depth_color_map, depth_intrinsic_matrix, min_dist):
        object_info_list = []
        image_height, image_width = depth_image.shape[:2]
        min_dist = min_dist / 1000.0
        if min_dist <= 0.3:  # 机械臂初始位置并不是与桌面水平，大于这个值说明已经低于桌面了，可能检测有误，不进行识别
            sphere_index = 0
            cuboid_index = 0
            cylinder_index = 0
            cylinder_horizontal_index = 0
            plane_values = utils.get_plane_values(depth_image, self.plane, depth_intrinsic_matrix)
            contours = utils.extract_contours(plane_values, 0.015)
            fx, fy = depth_intrinsic_matrix[0], depth_intrinsic_matrix[4]
            for obj in contours:
                area = cv2.contourArea(obj)
                if area < 300:
                    continue
                perimeter = cv2.arcLength(obj, True)  # 计算轮廓周长
                approx = cv2.approxPolyDP(obj, 0.035 * perimeter, True)  # 获取轮廓角点坐标

                CornerNum = len(approx)
                (cx, cy), radius = cv2.minEnclosingCircle(obj)
                #4.5版本定义为，x轴顺时针旋转最先重合的边为w，angle为x轴顺时针旋转的角度，angle取值为(0,90]
                center, (width, height), angle = cv2.minAreaRect(obj)
                # 将angle重定义为长边和x轴夹角
                if width > height:
                    long_edge_angle = angle
                else:
                    long_edge_angle = angle - 90
                # cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1, (0, 0, 255), 2) 
                depth = depth_image[int(cy), int(cx)]
                position = utils.calculate_world_position(cx, cy, depth, self.plane, self.endpoint, self.hand2cam_tf_matrix, depth_intrinsic_matrix)
                # self.get_logger().info(f'position: {position}') 
                gripper_open_external_width = int(fx / (depth / 1000.0) * (0.08 + 0.01)) # 爪子最大开口时外侧宽度
                gripper_open_external_height = int(fy / (depth / 1000.0) * 0.015)  
                gripper_inner_width = int(fx / (depth / 1000.0) * 0.055) # 爪子最大开口时内侧宽度 

                gripper_close_w, _ = utils.get_gripper_size(570)
                gripper_close_width = int(fx / (depth / 1000.0) * gripper_close_w)  
                
                # self.get_logger().info(f'gripper_close_w: {gripper_close_w} gripper_close_width: {gripper_close_width}')
                x, y, w, h = cv2.boundingRect(approx)
                mask = np.full((image_height, image_width), 0, dtype=np.uint8)
                
                cv2.drawContours(mask, [obj], -1, (255), cv2.FILLED) # 填充轮廓
                # cv2.imshow('mask', mask) 
                # 计算轮廓区域内像素值的标准差
                depth_std = np.nanstd(mask)
                objType = None
                # self.get_logger().info('\033[1;32m22222:%s\033[0m' % str([depth_std, CornerNum, abs(width/height - 1)]))
                
                if depth_std > 53.0 and CornerNum >= 4:
                    sphere_index += 1
                    angle = 0
                    long_edge_angle = 0
                    gripper_close_width = max(width, height) + 1
                    objType = 'sphere_' + str(sphere_index)
                elif depth_std > 41.0 :
                    cuboid_index += 1
                    objType = "cuboid_" + str(cuboid_index)
                else:
                    # self.get_logger().info('\033[1;32m11111: %s\033[0m' % str(abs(width/height - 1)))
                    if abs(width/height - 1) < 0.2:
                        distances = []
                        for point in obj:
                            px, py = point[0]  # 获取轮廓点的坐标
                            distance_to_circle = np.abs(np.sqrt((px - cx)**2 + (py - cy)**2) - radius)
                            distances.append(distance_to_circle)
                        
                        fit_error = np.std(distances)
                        # self.get_logger().info('\033[1;32m%s\033[0m' % str(fit_error)) 
                        if fit_error < 5:
                            cylinder_index += 1
                            angle = 0
                            long_edge_angle = 0
                            objType = "cylinder_" + str(cylinder_index)
                        else:
                            cuboid_index += 1
                            objType = "cuboid_" + str(cuboid_index)
                    else:
                        cylinder_horizontal_index += 1
                        objType = "cylinder_horizontal_" + str(cylinder_horizontal_index)
                # mask_rect1:angle mask_rect2:angle-90
                # self.get_logger().info('\033[1;32m%s\033[0m' % str(angle))
                mask_rect1, mask_rect2 = calculate_grasp_yaw_by_depth.get_gripper_masks(depth_image, center, long_edge_angle, gripper_open_external_width, gripper_open_external_height)  # 计算两个垂直方向上夹取的掩码和物体的掩码不相交的区域，即夹取的掩码区域, mask_rect1垂直短边的夹取姿态，mask_rect2长边
                mask = calculate_grasp_yaw_by_depth.get_obstacle_mask(depth_image, obj, plane_values, 0.09, position[2], self.gripper_depth) # 生成障碍物掩码
                mask1 = cv2.bitwise_and(mask_rect1, mask)  # 判断夹取姿态1时和周围物体有没有干涉
                mask2 = cv2.bitwise_and(mask_rect2, mask)  # 判断夹取姿态2时和周围物体有没有干涉
                angle_list = calculate_grasp_yaw_by_depth.calculate_obj_angles(width, height, gripper_inner_width, [long_edge_angle, np.any(mask1)], [long_edge_angle - 90, np.any(mask2)])  # 通过物体的尺寸来选择合适的夹取姿态
                yaw, gripper_angle = calculate_grasp_yaw_by_depth.calculate_grasp_angle(position, angle_list, long_edge_angle)
                if objType is not None:
                    if yaw is not None:
                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (gripper_open_external_width, gripper_open_external_height), gripper_angle)))], -1, (0, 255, 255), 2, cv2.LINE_AA)
                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (gripper_close_width, gripper_open_external_height), gripper_angle)))], -1, (255, 0, 255), 2, cv2.LINE_AA)
                        config_data = common.get_yaml_data(os.path.join(self.config_path, self.calibration_file))
                        offset = tuple(config_data['depth']['offset'])
                        scale = tuple(config_data['depth']['scale'])
                        for i in range(3):
                            position[i] = position[i] * scale[i]
                            position[i] = position[i] + offset[i]
                        index = int(objType.split('_')[-1])
                        obj_name = objType.split('_')[0]
                        object_info_list.append([obj_name, index, position, depth, [x, y, w, h, center, width, height, angle], bgr_image[int(center[1]), int(center[0])], yaw])
                    cv2.rectangle(depth_color_map, (x, y), (x + w, y + h), (255, 255, 255), 2)
        return object_info_list

    def main(self):
        while self.running:
            if self.enter: 
                try:
                    bgr_image, depth_image, camera_info, depth_camera_info = self.image_queue.get(block=True, timeout=1)
                except queue.Empty:
                    continue
                img = bgr_image.copy()
                try:
                    max_dist = 350
                    depth_image = utils.create_roi_mask(depth_image, bgr_image, self.corners, camera_info, self.extristric, max_dist, 0.08)
                    min_dist = utils.find_depth_range(depth_image, max_dist)
                    #像素值限制在0到350的范围内, 将深度图像的像素值限制和归一化到0到255的范围内
                    sim_depth_image = (1 - np.clip(depth_image, 0, max_dist).astype(np.float64) / max_dist) * 255
                        
                    depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)
                    if not self.start_transport and self.shapes is not None:
                        if self.enable_transport:
                            object_info_list = self.shape_recognition(bgr_image, depth_image, depth_color_map, depth_camera_info.k, min_dist)
                            reorder_object_info_list = object_info_list
                            if object_info_list:
                                if self.last_object_info_list:
                                    # 对比上一次的物体的位置来重新排序
                                    reorder_object_info_list = position_change_detect.position_reorder(object_info_list, self.last_object_info_list, 20)
                            if reorder_object_info_list:
                                # self.get_logger().info(f'{self.target_shape}')
                                if self.target_shape is None:
                                    # if self.shapes is not None:
                                    indices = [i for i, info in enumerate(reorder_object_info_list) if info[0] in self.shapes]
                                    # self.get_logger().info(f'indices: {indices}')
                                    if indices:
                                        min_depth_index = min(indices, key=lambda i: reorder_object_info_list[i][3])
                                        self.target_shape = reorder_object_info_list[min_depth_index][0]

                                else:
                                    target_index = [i for i, info in enumerate(reorder_object_info_list) if info[0] == self.target_shape]
                                    if target_index:
                                        target_index = target_index[0]
                                        obejct_info = reorder_object_info_list[target_index]
                                        x, y, w, h, center, width, height, angle = obejct_info[4]
                                        
                                        # self.get_logger().info(f'{obejct_info}')
                                        cv2.putText(depth_color_map, obejct_info[0] + str(obejct_info[1]), (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                        (0, 0, 0), 2, cv2.LINE_AA)
                                        cv2.putText(depth_color_map, obejct_info[0] + str(obejct_info[1]), (x + w // 2, y + (h // 2) - 10), cv2.FONT_HERSHEY_COMPLEX, 1.0,
                                                        (255, 255, 255), 1)
                                        cv2.drawContours(depth_color_map, [np.int0(cv2.boxPoints((center, (width, height), angle)))], -1,
                                                             (0, 0, 255), 2, cv2.LINE_AA)
                                        position = obejct_info[2]
                                        e_distance = round(math.sqrt(pow(self.last_position[0] - position[0], 2)) + math.sqrt(
                                                pow(self.last_position[1] - position[1], 2)), 5)
                                        if e_distance <= 0.005:
                                            self.count_move = 0
                                            self.count_still += 1
                                        else:
                                            self.count_move += 1
                                            self.count_still = 0
                                            
                                        if self.count_still > 10:
                                            self.count_still = 0
                                            self.count_move = 0
                                            # self.get_logger().info(f'{obejct_info}')
                                            self.start_transport = True
                                            self.transport_info = obejct_info
                                        self.last_position = position
                                    else:
                                        self.target_shape = None
                            self.last_object_info_list = reorder_object_info_list
                    # display_image = bgr_image[40:440, ]
                    zero_color = depth_color_map[sim_depth_image == 0][0]  # 取第一个值，假设颜色一致
                    depth_color_map_padded = cv2.copyMakeBorder(
                        depth_color_map,
                        40,    # 上方填充
                        40, # 下方填充
                        0,              # 左方填充
                        0,              # 右方填充
                        borderType=cv2.BORDER_CONSTANT,
                        value=tuple(map(int, zero_color))  # 填充颜色
                    )
                    if self.display_rgb:
                        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(bgr_image, "bgr8"))
                    else:
                        self.result_publisher.publish(self.bridge.cv2_to_imgmsg(depth_color_map_padded, "bgr8"))
                    if self.display:
                        result_image = np.concatenate([bgr_image, depth_color_map_padded], axis=1)
                        cv2.imshow("depth", result_image)
                        cv2.waitKey(1)
                except Exception as e:
                    self.get_logger().info(str(e))
            else:
                time.sleep(0.1)

    def multi_callback(self, ros_rgb_image, ros_depth_image, camera_info, depth_camera_info):
        rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
        bgr_image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((bgr_image, depth_image, camera_info, depth_camera_info))

def main():
    node = ShapeRecognitionNode('shape_recognition')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.running = False  # 停止线程标志
        executor.shutdown()

if __name__ == "__main__":
    main()


