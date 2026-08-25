#!/usr/bin/env python3
# coding: utf8
#颜色分拣

import os
import cv2
import yaml
import time
import math
import queue
import rclpy 
import signal
import threading
import numpy as np
from rclpy.node import Node
from sdk import common, fps
from cv_bridge import CvBridge
from interfaces.srv import SetStringBool
from std_srvs.srv import Trigger, SetBool
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition 
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position
from ros_robot_controller_msgs.msg import MotorsState, MotorState
from servo_controller.action_group_controller import ActionGroupController

PLACE_POSITION = {
    'red': [0.01, -0.15, 0.01],
    'green': [-0.07, -0.15, 0.01],
    'blue': [-0.15, 0.15, 0.01],
}    

class ColorSortingNode(Node):

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.running = True
        self.start_count = False
        self.lock = threading.RLock()
        self.image_queue = queue.Queue(maxsize=2)
        self.fps = fps.FPS()    # 帧率统计器(frame rate counter)
        self.bridge = CvBridge()  # 用于ROS Image消息与OpenCV图像之间的转换
        self.center_imgpts = None
        self.offset = 0.005 #夹取位置偏差调节
        self.pick_pitch = 80
        self.count = 0
        self.data = common.get_yaml_data("/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/lab_config.yaml")  
        self.lab_data = self.data['/**']['ros__parameters'] 
        self.camera_type = os.environ['CAMERA_TYPE']
        self.min_area = 500
        self.max_area = 15000 
        self.target = None
        self.image_sub = None
        self.controller = ActionGroupController(self.create_publisher(ServosPosition, 'servo_controller', 1), '/home/ubuntu/factory_utils/arm_pc/ActionGroups')

        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 1)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)
        timer_cb_group = ReentrantCallbackGroup()
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose', callback_group=timer_cb_group)
        self.get_current_pose_client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()
        threading.Thread(target=self.main).start()

    def init_processing(self):
        set_servo_position(self.joints_pub, 1.0, ((1, 500), (2, 630), (3, 90), (4, 85), (5, 480), (10, 200)))
        time.sleep(1.0)
        # self.set_motor(1, -100.0)

    def set_motor(self,id, rps):
        data = MotorState()
        data.id = id
        data.rps = rps
        msg = MotorsState()
        msg.data.append(data)
        self.motor_pub.publish(msg)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def adaptive_threshold(self, gray_image):
        # 用自适应阈值先进行分割, 过滤掉侧面
        # cv2.ADAPTIVE_THRESH_MEAN_C： 邻域所有像素点的权重值是一致的
        # cv2.ADAPTIVE_THRESH_GAUSSIAN _C ： 与邻域各个像素点到中心点的距离有关，通过高斯方程得到各个点的权重值
        binary = cv2.adaptiveThreshold(gray_image, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 41, 7)
        return binary
    
    def canny_proc(self, bgr_image):
        # 边缘提取，用来进一步分割(当两个相同颜色物体靠在一起时，只能靠边缘去区分)
        mask = cv2.Canny(bgr_image, 9, 41, 9, L2gradient=True)
        mask = 255 - cv2.dilate(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (11, 11)))  # 加粗边界，黑白反转
        return mask

    def get_top_surface(self, rgb_image):
        # 为了只提取物体最上层表面
        # 将图像转换到HSV颜色空间
        image_scale = cv2.convertScaleAbs(rgb_image, alpha=2.5, beta=0)
        image_gray = cv2.cvtColor(image_scale, cv2.COLOR_RGB2GRAY)
        image_mb = cv2.medianBlur(image_gray, 3)  # 中值滤波
        image_gs = cv2.GaussianBlur(image_mb, (5, 5), 5)  # 高斯模糊去噪
        binary = self.adaptive_threshold(image_gs)  # 阈值自适应
        mask = self.canny_proc(image_gs)  # 边缘检测
        mask1 = cv2.bitwise_and(binary, mask)  # 合并两个提取出来的图像，保留他们共有的地方
        roi_image_mask = cv2.bitwise_and(rgb_image, rgb_image, mask=mask1)  # 和原图做遮罩，保留需要识别的区域
        return roi_image_mask
 
    def image_callback(self, ros_image):
        # 将ros格式图像转换为opencv格式
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # # 将图像放入队列
        self.image_queue.put(rgb_image)
 
    def image_processing(self):
        rgb_image = self.image_queue.get()
        result_image = np.copy(rgb_image)
        target_list = []
        index = 0
        if self.start_count :
            img_h, img_w = rgb_image.shape[:2]

            rgb_image = self.get_top_surface(rgb_image)
            image_lab = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间(convert to LAB space)
            for i in ['red', 'green', 'blue']:

                mask = cv2.inRange(image_lab, tuple(self.lab_data['color_range_list'][i]['min']), tuple(self.lab_data['color_range_list'][i]['max']))  # 二值化
                
                # 平滑边缘，去除小块，合并靠近的块
                eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
                contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出所有轮廓(find all contours)
                contours_area = map(lambda c: (math.fabs(cv2.contourArea(c)), c), contours)  # 计算轮廓面积(calculate contour area)
                contours = map(lambda a_c: a_c[1], filter(lambda a: self.min_area <= a[0] <= self.max_area, contours_area))
                for c in contours:
                    area = math.fabs(cv2.contourArea(c))
                    rect = cv2.minAreaRect(c)  # 获取最小外接矩形
                    (center_x, center_y), _ = cv2.minEnclosingCircle(c)
                    center_x, center_y =  center_x,  center_y
                    cv2.circle(result_image, (int(center_x), int(center_y)), 8, (0, 0, 0), -1)
                    corners = list(map(lambda p: (p[0],  p[1]), cv2.boxPoints(rect))) # 获取最小外接矩形的四个角点, 转换回原始图的坐标
                    cv2.drawContours(result_image, [np.intp(corners)], -1, (0, 255, 255), 2, cv2.LINE_AA)  # 绘制矩形轮廓
                    index += 1 # 序号递增
                    angle = int(round(rect[2]))  # 矩形角度
                    target_list.append([i, index, (center_x, center_y), angle])
                    
                    # self.get_logger().info('\033[1;32m%s\033[0m' %center_y)
        return target_list, result_image

    def move(self, center_y):
        if self.running:
            x =  ((center_y - 235) / (335 -235)) * (0.11 - 0.14) + 0.14 #根据识别轮廓的中心点确定色块在机械臂X轴上的距离
            x = x + self.offset
            position = [x, 0, 0.08]

            if self.running:
                position[2] += 0.02
                msg = set_pose_target(position, self.pick_pitch,  [-180.0, 180.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                servo_data = res.pulse  
                set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]), ))
                time.sleep(1)
                set_servo_position(self.joints_pub, 1.0, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
                time.sleep(1)
                

            if self.running:
                position[2] -= 0.03
                msg = set_pose_target(position, self.pick_pitch,  [-90.0, 90.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                servo_data = res.pulse

                set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
                time.sleep(1)

                set_servo_position(self.joints_pub, 1.0, ((10, 600),))
                time.sleep(1)
                
            if self.running:
                position[2] += 0.04
                msg = set_pose_target(position, self.pick_pitch,  [-180.0, 180.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                servo_data = res.pulse  

                set_servo_position(self.joints_pub, 1.0, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
                time.sleep(1)

                position =  PLACE_POSITION[self.target[0]] #获取放置位置

            if self.running:
                position[2] += 0.03
                msg = set_pose_target(position, self.pick_pitch,  [-90.0, 90.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                    
                servo_data = res.pulse  
                set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]),))
                time.sleep(1)
                set_servo_position(self.joints_pub, 1.0, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
                time.sleep(1)

            if self.running:
                position[2] -= 0.03
                msg = set_pose_target(position, self.pick_pitch,  [-90.0, 90.0], 1.0)
                res = self.send_request(self.kinematics_client, msg)
                servo_data = res.pulse 

                self.get_logger().info('\033[1;32m%s\033[0m' %servo_data)
                set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
                time.sleep(1)
                set_servo_position(self.joints_pub, 1.0, ((10, 200),))
                time.sleep(1)

            if self.running:
                set_servo_position(self.joints_pub, 1.0, ( (2, 630), (3, 90), (4, 85), (5, 480), (10, 200)))
                time.sleep(1.0)

                set_servo_position(self.joints_pub, 1.0, ((1, 500),))  
                time.sleep(1.0)
                self.target = None
                self.start_count = True

            if not self.running:

                set_servo_position(self.joints_pub, 1.0, ((2, 630), (3, 90), (4, 85), (5, 480), (10, 200)))
                time.sleep(1.0)
                set_servo_position(self.joints_pub, 1.0, ((1, 500),))  
                time.sleep(1.0)
                self.target = None

    def main(self):
        while True:

            target_list, result_image = self.image_processing()
            if target_list and self.running:
                if self.target is not None :
                    if self.target[0] == target_list[0][0]:
                        self.count += 1
                    else:
                        self.target = target_list[0]
                        self.count = 0
                else :
                    self.target = target_list[0]
            if self.count > 40:
                self.count = 0

                self.start_count = False
                threading.Thread(target=self.move,args=(target_list[0][2][1],), daemon=True).start()

            if result_image is not None :
                self.fps.update()
                result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)
                cv2.imshow('result_image', result_image)
                key = cv2.waitKey(1)
                if key != -1:
                    if key == 97: #A
                        self.running = True
                        self.start_count = True
                        self.init_processing()
                    if key == 115: # S
                        self.running = False
                        self.count = 0
                        self.set_motor(1, 0.0)


def main():
    node = ColorSortingNode('color_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()



