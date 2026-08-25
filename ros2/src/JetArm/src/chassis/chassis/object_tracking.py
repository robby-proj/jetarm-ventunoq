#!/usr/bin/env python3
# encoding: utf-8
# 颜色跟踪(color tracking)
import os
import cv2
import math
import time
import queue
import rclpy
import threading
import numpy as np
import sdk.pid as pid
import sdk.misc as misc
import sdk.common as common
from chassis import mecanum 
from rclpy.node import Node
from app.common import Heart
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from app.common import ColorPicker
from chassis_msgs.msg import Mecanum
from std_srvs.srv import SetBool, Trigger
from interfaces.srv import SetPoint, SetFloat64
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from servo_controller.bus_servo_control import set_servo_position

class ObjectTracker:
    def __init__(self, color, node):
        self.node = node
        self.chassis_type = os.environ['CHASSIS_TYPE']
        self.camera_type = os.environ['CAMERA_TYPE']
        self.pid_yaw = pid.PID(0.01, 0.0, 0.001)
        self.pid_dist = pid.PID(0.002, 0.0, 0.00)
        self.last_color_circle = None
        self.lost_target_count = 0
        self.target_lab, self.target_rgb = color
        self.weight_sum = 1.0
        self.x_stop = 320
        self.Mecanum = mecanum.MecanumChassis()
        if self.camera_type == 'GEMINI':
            self.y_stop = 300
            self.pro_size = (640, 480)
        else:
            self.y_stop = 300
            self.pro_size = (640, 480)

    def __call__(self, image, result_image, threshold):
        mecanum = Mecanum()
        velocity_x = 0.0
        velocity_y = 0.0
        angular_rate = 0.0
        h, w = image.shape[:2]
        image = cv2.resize(image, self.pro_size)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2LAB)  # RGB转LAB空间(convert RGB to LAB space)
        image = cv2.GaussianBlur(image, (5, 5), 5)

        min_color = [int(self.target_lab[0] - 50 * threshold * 2),
                     int(self.target_lab[1] - 50 * threshold),
                     int(self.target_lab[2] - 50 * threshold)]
        max_color = [int(self.target_lab[0] + 50 * threshold * 2),
                     int(self.target_lab[1] + 50 * threshold),
                     int(self.target_lab[2] + 50 * threshold)]
        target_color = self.target_lab, min_color, max_color
        mask = cv2.inRange(image, tuple(target_color[1]), tuple(target_color[2]))  # 二值化(binarization)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 腐蚀(erode)
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))  # 膨胀(dilate)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]  # 找出轮廓(find contours)
        contour_area = map(lambda c: (c, math.fabs(cv2.contourArea(c))), contours)  # 计算各个轮廓的面积(calculate the area of each contour)
        contour_area = list(filter(lambda c: c[1] > 40, contour_area))  # 剔除>面积过小的轮廓(remove contours with area that is too small)
        circle = None
        if len(contour_area) > 0:
            if self.last_color_circle is None:
                contour, area = max(contour_area, key=lambda c_a: c_a[1])
                circle = cv2.minEnclosingCircle(contour)
            else:
                (last_x, last_y), last_r = self.last_color_circle
                circles = map(lambda c: cv2.minEnclosingCircle(c[0]), contour_area)
                circle_dist = list(map(lambda c: (c, math.sqrt(((c[0][0] - last_x) ** 2) + ((c[0][1] - last_y) ** 2))),
                                       circles))
                circle, dist = min(circle_dist, key=lambda c: c[1])
                if dist < 100:
                    circle = circle
        if circle is not None:
            self.lost_target_count = 0
            (x, y), r = circle
            x = x / self.pro_size[0] * w
            y = y / self.pro_size[1] * h
            r = r / self.pro_size[0] * w

            cv2.circle(result_image, (self.x_stop, self.y_stop), 5, (255, 255, 0), -1)
            result_image = cv2.circle(result_image, (int(x), int(y)), int(r), (self.target_rgb[0],
                                                                               self.target_rgb[1],
                                                                               self.target_rgb[2]), 2)
            vx = 0
            vw = 0
            if abs(y - self.y_stop) > 20:
                self.pid_dist.update(y - self.y_stop)
                velocity_y = misc.map(common.set_range(self.pid_dist.output, -0.35, 0.35), -0.35, 0.35, -150, 150)
            else:
                self.pid_dist.clear()
            if abs(x - self.x_stop) > 20:
                self.pid_yaw.update(x - self.x_stop)
                if self.chassis_type == 'Mecanum':
                    velocity_x = misc.map(common.set_range(self.pid_yaw.output, -2, 2), -2, 2, 150, -150)
                elif self.chassis_type == 'Tank':
                    angular_rate = misc.map(common.set_range(self.pid_yaw.output, -2, 2), -2, 2, -150, 150)
            else:
                self.pid_yaw.clear()

            v, d = self.Mecanum.translation(velocity_x, velocity_y, fake=True)
            mecanum.velocity = v
            mecanum.direction = d
            mecanum.angular_rate = angular_rate
        return result_image, mecanum

class OjbectTrackingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        self.set_callback = False
        self.color_picker = None
        self.tracker = None
        self.is_running = False
        self.threshold = 0.5
        self.dist_threshold = 0.3
        self.lock = threading.RLock()
        self.image_sub = None
        self.result_image = None
        self.image_height = None
        self.image_width = None
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(2)
        self.chassis_pub = self.create_publisher(Mecanum, '/chassis_controller/command', 1)
        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
                
        self.timer_cb_group = ReentrantCallbackGroup()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish', callback_group=self.timer_cb_group)
        self.client.wait_for_service()

        self.set_running_srv = self.create_service(SetBool, '~/set_running', self.set_running_srv_callback)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.timer = self.create_timer(0.0, self.init_process, callback_group=self.timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        set_servo_position(self.joints_pub, 1, ((10, 300), (5, 500), (4, 210), (3, 40), (2, 665), (1, 500)))
        time.sleep(1.0)
        threading.Thread(target=self.main, daemon=True).start()

    def main(self):
        while True:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                continue

            result = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            cv2.imshow("result", result)
            if not self.set_callback:
                self.set_callback = True
                # 设置鼠标点击事件的回调函数(set callback function for mouse clicking event)
                cv2.setMouseCallback("result", self.mouse_callback)
            key = cv2.waitKey(1)
        self.chassis_pub.publish(Mecanum())
        rclpy.shutdown()
        
    def mouse_callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            msg = SetPoint.Request()
            if self.image_height is not None and self.image_width is not None:
                msg.data.x = x / self.image_width
                msg.data.y = y / self.image_height
                self.set_target_color_srv_callback(msg, SetPoint.Response())

    def set_target_color_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set_target_color')
        with self.lock:
            x, y = request.data.x, request.data.y
            if x == -1 and y == -1:
                self.color_picker = None
                self.tracker = None
            else:
                self.tracker = None
                self.color_picker = ColorPicker(request.data, 10)
            self.chassis_pub.publish(Mecanum())
        response.success = True
        response.message = "set_target_color"
        return response

    def set_running_srv_callback(self, request, response):
        with self.lock:
            self.is_running = request.data
            if self.is_running:
                self.get_logger().info('\033[1;32m%s\033[0m' % 'set_running')
            else:
                self.chassis_pub.publish(Mecanum())
                self.get_logger().info('\033[1;32m%s\033[0m' % 'stop')
        response.success = True
        response.message = "set_running"
        return response

    def image_callback(self, ros_image):
        # 将ros格式(rgb)转为opencv的rgb格式(convert RGB format of ROS to that of OpenCV)
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "rgb8")
        rgb_image = np.array(cv_image, dtype=np.uint8)
        self.image_height, self.image_width = rgb_image.shape[:2]

        result_image = np.copy(rgb_image)  # 显示结果用的画面(the image used for display the result)
        with self.lock:
            # 颜色拾取器和识别追踪互斥, 如果拾取器存在就开始拾取(color picker and object tracking are mutually exclusive. If the color picker exists, start picking colors)
            if self.color_picker is not None:  # 拾取器存在(color pick exists)
                target_color, result_image = self.color_picker(rgb_image, result_image)
                if target_color is not None:
                    self.color_picker = None
                    self.tracker = ObjectTracker(target_color, self)
            else:
                if self.tracker is not None:
                    try:
                        result_image, mecanum = self.tracker(rgb_image, result_image, self.threshold)
                        if self.is_running:
                            self.chassis_pub.publish(mecanum)
                        else:
                            self.tracker.pid_dist.clear()
                            self.tracker.pid_yaw.clear()
                    except Exception as e:
                        self.get_logger().error(str(e))
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
            # 将图像放入队列(put the image into the queue)
        self.image_queue.put(result_image)

def main():
    node = OjbectTrackingNode('object_tracking')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

