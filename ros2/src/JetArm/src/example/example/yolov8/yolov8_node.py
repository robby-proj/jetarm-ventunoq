#!/usr/bin/python3
#coding=utf8
# YOLOv8 OBB识别节点

import os
import time
import math
import queue
import rclpy
import ctypes
import signal
import threading
import numpy as np
import sdk.fps as fps
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from interfaces.msg import ObjectInfo, ObjectsInfo
from example.yolov8.yolov8_trt import YoLov8TRT,plot_one_box

MODE_PATH = os.path.split(os.path.realpath(__file__))[0]
class Colors:
    # Ultralytics color palette https://ultralytics.com/
    def __init__(self):
        hex = ('FF3838', 'FF9D97', 'FF701F', 'FFB21D', 'CFD231', '48F90A', '92CC17', '3DDB86', '1A9334', '00D4BB',
               '2C99A8', '00C2FF', '344593', '6473FF', '0018EC', '8438FF', '520085', 'CB38FF', 'FF95C8', 'FF37C7')
        self.palette = [self.hex2rgb('#' + c) for c in hex]
        self.n = len(self.palette)

    def __call__(self, i, bgr=False):
        c = self.palette[int(i) % self.n]
        return (c[2], c[1], c[0]) if bgr else c

    @staticmethod
    def hex2rgb(h):  # rgb order (PIL)
        return tuple(int(h[1 + i:1 + i + 2], 16) for i in (0, 2, 4))
colors = Colors()  # create instance for 'from utils.plots import colors'

# class YOLOv8Node:
class Yolov8Node(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.name = name
        # self.image_sub = None
        self.start = False
        # self.bgr_image = None
        self.running = True

        self.start_time = time.time()
        self.frame_count = 0
        self.fps = fps.FPS()  # fps计算器


        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)

        signal.signal(signal.SIGINT, self.shutdown)

        lib = self.get_parameter('lib').value
        engine = self.get_parameter('engine').value
        self.classes = self.get_parameter('classes').value

        
        ctypes.CDLL(os.path.join(MODE_PATH, lib))
        self.yolov8_wrapper = YoLov8TRT(os.path.join(MODE_PATH, engine))
        
        self.create_service(Trigger, '/yolov8/start', self.start_srv_callback)  # 进入玩法
        self.create_service(Trigger, '/yolov8/stop', self.stop_srv_callback)  # 退出玩法

        self.image_sub = self.create_subscription(Image, '/depth_cam/rgb/image_raw', self.image_callback, 1)

        self.object_pub = self.create_publisher(ObjectsInfo, '~/object_detect', 1)
        self.result_image_pub = self.create_publisher(Image, '~/object_image', 1)
        threading.Thread(target=self.image_proc, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())


    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def get_node_state(self, request, response):
        response.success = True
        return response

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start yolov8 detect")

        self.start = True
        response.success = True
        response.message = "start"
        return response



    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop yolov8 detect")

        self.start = False
        response.success = True
        response.message = "start"
        return response

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # 将图像放入队列
        self.image_queue.put(bgr_image)
   
    def shutdown(self, signum, frame):
        self.running = False
        self.get_logger().info('\033[1;32m%s\033[0m' % "shutdown")

    def image_proc(self):
        while self.running:
            try:
                result_image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue

            try:
                if self.start:
                    objects_info = []
                    h, w = result_image.shape[:2]
                    boxes, scores, classid = self.yolov8_wrapper.infer(result_image)
                    for box, cls_conf, cls_id in zip(boxes, scores, classid):
                        color = colors(cls_id, True)
                        angle_in_degrees = int(math.degrees(box[4]))
                        plot_one_box(
                                        box,
                                        result_image,
                                        label="{}:{:.2f}".format(self.classes[cls_id], cls_conf),
                                        color=color,
                                        line_thickness=3,
                                        rotated=True)
                        object_info = ObjectInfo()
                        object_info.class_name = self.classes[cls_id]
                        object_info.box = box.astype(int).tolist()
                        object_info.width = w
                        object_info.height = h
                        object_info.score = float(cls_conf)
                        object_info.angle = angle_in_degrees
                        # self.get_logger().info('angle: ' + str(angle_in_degrees))
                        objects_info.append(object_info)  


                    object_msg = ObjectsInfo()
                    object_msg.objects = objects_info
                    self.object_pub.publish(object_msg)
            except BaseException as e:
                print(e)

            self.fps.update()
            result_image = self.fps.show_fps(result_image)
            self.result_image_pub.publish(self.bridge.cv2_to_imgmsg(result_image, "bgr8"))
        else:
            time.sleep(0.01)

        self.yolov8_wrapper.destroy() 
        rclpy.shutdown()


def main():
    node = Yolov8Node('yolov8')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
