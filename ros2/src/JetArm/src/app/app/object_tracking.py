#!/usr/bin/env python3
# encoding: utf-8
# 目标跟踪

import os
import cv2
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.fps as fps
from math import radians
from rclpy.node import Node
import sdk.common as common
from app.common import Heart
from cv_bridge import CvBridge
import app.face_tracker as face_tracker
import app.color_tracker as color_tracker
from std_srvs.srv import Trigger, SetBool
from kinematics_msgs.srv import SetRobotPose
from sensor_msgs.msg import Image , CameraInfo
from interfaces.srv import SetPoint, SetFloat64
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from servo_controller.bus_servo_control import set_servo_position


class ObjectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.last_time = 0
        self.current_time = 0
        self.name = name
        self.current_pose = None
        self.tracker = None
        self.enable_color_tracking = False
        self.enable_face_tracking = False
        self.fps = fps.FPS()
        self.bridge = CvBridge()
        self.lock = threading.RLock()
        self.image_queue = queue.Queue(maxsize=2)
        self.thread_started = True 
        self.thread = None
        self.x_init = 0.18
        
        self.image_sub = None
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        self.result_publisher = self.create_publisher(Image, '~/image_result',  1)
        self.enter_srv = self.create_service(Trigger, '~/enter', self.enter_srv_callback)
        self.exit_srv = self.create_service(Trigger, '~/exit', self.exit_srv_callback)
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        Heart(self, self.name + '/heartbeat', 5, lambda _: self.exit_srv_callback(request=Trigger.Request(), response=Trigger.Response()))  # 心跳包(heartbeat package)
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()
        self.enable_detect_srv = self.create_service(SetBool, '~/enable_color_tracking', self.enable_color_srv_callback)
        self.enable_transport_srv = self.create_service(SetBool, '~/enable_face_tracking', self.enable_face_srv_callback)
        self.set_target_color_srv = self.create_service(SetFloat64, '~/set_target_color', self.set_target_color_srv_callback)

        if self.get_parameter('start').value:
            threading.Thread(target=self.enter_srv_callback,args=(Trigger.Request(), Trigger.Response()), daemon=True).start()
            time.sleep(1.0)
            req = SetBool.Request()
            req.data = True 
            res = SetBool.Response()
            self.enable_color_srv_callback(req, res)

        if not self.get_parameter('broadcast').value: 
            req = SetFloat64.Request()
            req.data = 1.0 #追踪的颜色 0.0为不追踪， 1.0为红色， 2.0为绿色，3.0为蓝色
            res = SetFloat64.Response()
            self.set_target_color_srv_callback(req, res)
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'init finish')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def goback(self):
        set_servo_position(self.joints_pub, 1.5, ((10, 200), (5, 500), (4,300), (3, 85), (2,730), (1, 500)))
        time.sleep(1.5)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
    def to_face_tracking_base(self):
        with self.lock:
            thread = threading.Thread(target=self.goback).start()
            if self.enable_color_tracking:
                self.enable_color_tracking = False
                self.tracker = None
            self.tracker = face_tracker.FaceTracker()
            self.enable_face_tracking = True
            self.thread = None
            self.get_logger().info('\033[1;32m%s\033[0m' % "start color tracking")

    def to_color_tracking_base(self):
        with self.lock:
            thread = threading.Thread(target=self.goback).start()
            if self.enable_face_tracking:
                self.enable_face_tracking = False
                self.tracker = None
            self.enable_color_tracking = True
            self.thread = None
        self.get_logger().info('\033[1;32m%s\033[0m' % "start color tracking")

    def enable_color_srv_callback(self, request, response):
        with self.lock:
            if request.data:
                if self.thread is None:
                    self.get_logger().info('\033[1;32m%s\033[0m' % "start color tracking...")
                    self.thread = threading.Thread(target=self.to_color_tracking_base)
                    self.thread.start()
                    response.success = True
                    response.message = "enter"
                    return response
                else:
                    msg = "Enable Color Tracking, 有其他操作正在进行, 请稍后重试"
                    self.get_logger().info(msg)
                    response.success = True
                    response.message = "enter"
                    return response
            else:
                self.get_logger().info('\033[1;32m%s\033[0m' % "start color tracking...")
                if self.enable_color_tracking:
                    self.enable_color_tracking = False
                    self.tracker = None
                response.success = True
                response.message = "enter"
                return response

    def enable_face_srv_callback(self, request, response):
        with self.lock:
            if self.thread is None:
                if request.data:
                    self.get_logger().info('\033[1;32m%s\033[0m' % "start face tracking...")
                    self.thread = threading.Thread(target=self.to_face_tracking_base)
                    self.thread.start()
                else:
                    self.get_logger().info('\033[1;32m%s\033[0m' % "stop face tracking...")
                    if self.enable_face_tracking:
                        self.enable_face_tracking = False
                        self.tracker = None
                response.success = True
                response.message = "enter"
                return response
            else:
                msg = "Enable Face Tracking, 有其他操作正在进行, 请稍后重试"
                self.get_logger().info(msg)
                response.success = True
                response.message = "enter"
                return response

    def set_target_color_srv_callback(self, request, response):
        with self.lock:
            if request.data != 0:
                colors = ["None","red", "green", "blue"]
                self.get_logger().info("\033[1;32mset target color: " + colors[int(request.data)] + "\033[0m")
                self.tracker = color_tracker.ColorTracker(colors[int(request.data)])
                if self.enable_face_tracking:
                    self.enable_face_tracking = False
                    self.tracker = None
            else:
                self.tracker = None
            response.success = True
            response.message = "enter"
            return response

    def enter_srv_callback(self, request, response):
        # 获取和发布图像的topic(get and publish the topic of image)
        self.get_logger().info('\033[1;32m%s\033[0m' % "loading object tracking")
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        with self.lock:
            self.enable_face_tracking = False
            self.enable_color_tracking = False
        self.thread_started = False
        response.success = True
        response.message = "enter"
        return response

    def exit_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "exit object tracking")
        with self.lock:
            self.enable_face_tracking = False
            self.enable_color_tracking = False
            self.tracker = None
        try:
            if self.image_sub is not None:
                self.destroy_subscription(self.image_sub)
                self.image_sub = None
        except Exception as e:
            self.get_logger().error(str(e))

        response.success = True
        response.message = "exit"
        return response

    # def tracking(self, p_y):
        # msg = set_pose_target([0.15, 0, 0.26], 0.0, [-180.0, 180.0], 1.0)
        # res = self.send_request(self.kinematics_client, msg)
        # if res.pulse:
            # servo_data = res.pulse
            # set_servo_position(self.joints_pub, 0.02, ( (1, int(p_y[1])), ))


    def image_callback(self, ros_image):
        # 将ros格式图像转换为opencv格式(convert the ros format image to opencv format)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        with self.lock:
            if not self.thread_started:
                # 只在第一次调用时启动线程
                threading.Thread(target=self.image_processing, args=(ros_image,), daemon=True).start()
                self.thread_started = True 

        if self.image_queue.full():
            # # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # # 将图像放入队列
        self.image_queue.put(rgb_image)


    def image_processing(self, ros_image):
        while True:
            rgb_image = self.image_queue.get()
            result_image = np.copy(rgb_image)
            if self.thread is None and self.tracker is not None:
                if self.tracker.tracker_type == 'color' and self.enable_color_tracking:
                    result_image, p_y = self.tracker.proc(rgb_image, result_image)
                    if p_y is not None:
                        msg = set_pose_target([0.15, 0, p_y[0]], 0.0, [-180.0, 180.0], 1.0)
                        res = self.send_request(self.kinematics_client, msg)
                        if res.pulse:
                            servo_data = res.pulse
                            set_servo_position(self.joints_pub, 0.02, ( (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, int(p_y[1])) ))
                elif self.tracker.tracker_type == 'face' and self.enable_face_tracking:
                    result_image, p_y = self.tracker.proc(rgb_image, result_image)
                    if p_y is not None:
                        set_servo_position(self.joints_pub, 0.02, ((1, p_y[1]), (4, p_y[0])))

            self.result_publisher.publish(self.bridge.cv2_to_imgmsg(result_image, "rgb8"))
            if result_image is not None and self.get_parameter('display').value:
                result_image = cv2.cvtColor(result_image, cv2.COLOR_BGR2RGB)
                cv2.imshow('result_image', result_image)
                key = cv2.waitKey(1)
def main():
    node = ObjectTrackingNode('object_tracking')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
     
if __name__ == "__main__":
    main()

