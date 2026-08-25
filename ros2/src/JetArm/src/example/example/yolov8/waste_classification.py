#!/usr/bin/env python3
# encoding: utf-8
# 垃圾分类
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
import sdk.fps as fps
from sdk import common
from rclpy.node import Node
from cv_bridge import CvBridge
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist
from interfaces.msg import ObjectsInfo
from xf_mic_asr_offline import voice_play
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position

WASTE_CLASSES = {
    'food_waste': ('BananaPeel', 'BrokenBones', 'Ketchup'),
    'hazardous_waste': ('Marker', 'OralLiquidBottle', 'StorageBattery'),
    'recyclable_waste': ('PlasticBottle', 'Toothbrush', 'Umbrella'),
    'residual_waste': ('Plate', 'CigaretteEnd', 'DisposableChopsticks'),
}

POSITIONS = {
    'food_waste': (0.085, -0.24, 0.025),
    'hazardous_waste': (0.017, -0.24, 0.025),
    'recyclable_waste': (-0.05, -0.238, 0.025),
    'residual_waste': (-0.112, -0.233, 0.025),
        }

class WasteClassificationNode(Node):
    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.011],
    [0.0, -1.0, 0.0, 0.045],
    [0.0, 0.0, 0.0, 1.0]
]

    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.running = True
        self.center = None
        self.count = 0
        self.class_name = None
        self.start_place = False
        self.start_move = False
        self.start_count = False

        self.config_file = 'transform.yaml'
        self.config_path = "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/"
        self.pick_pitch = 80
        self.current_class_name = None
        self.fps = fps.FPS()  # fps计算器
        self.language = os.environ['ASR_LANGUAGE']
        with open(self.config_path + self.config_file, 'r') as f:
            config = yaml.safe_load(f)

            # 转换为 numpy 数组
            extristric = np.array(config['extristric'])
            white_area_center = np.array(config['white_area_pose_world'])
        self.white_area_center = white_area_center
        tvec = extristric[:1]  # 平移向量
        rmat = extristric[1:]  # 旋转矩阵
        tvec, rmat = common.extristric_plane_shift(np.array(tvec).reshape((3, 1)), np.array(rmat), 0.030)
        self.extristric = tvec, rmat
        self.previous_pose = None  # 上一次检测到的位置


        signal.signal(signal.SIGINT, self.shutdown)
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback, callback_group=timer_cb_group)  # 进入玩法
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group)  # 退出玩法
        self.create_subscription(Image, '/yolov8/object_image', self.image_callback, 1)
        self.create_subscription(ObjectsInfo, '/yolov8/object_detect', self.get_object_callback, 1)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/depth_cam/depth/camera_info', self.camera_info_callback, 1)
        self.start_yolov8_client = self.create_client(Trigger, '/yolov8/start', callback_group=timer_cb_group)
        self.start_yolov8_client.wait_for_service()
        self.stop_yolov8_client = self.create_client(Trigger, '/yolov8/stop', callback_group=timer_cb_group)
        self.stop_yolov8_client.wait_for_service()

        self.debug = self.get_parameter('debug').value
        self.broadcast = self.get_parameter('broadcast').value

        
        #等待服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()

        set_servo_position(self.joints_pub, 1.0, ((1, 500), (2, 560), (3, 130), (4, 115), (5, 500), (10, 200)))  # 设置机械臂初始位置
        time.sleep(1)

        if self.get_parameter('start').value:
            self.start_srv_callback(Trigger.Request(), Trigger.Response())

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def camera_info_callback(self, msg):
        self.K = np.matrix(msg.k).reshape(1, -1, 3)

    def play(self, name):
        if self.broadcast:
            voice_play.play(name, language=self.language)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start garbage classification")

        self.send_request(self.start_yolov8_client, Trigger.Request())
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop garbage classification")
        self.send_request(self.stop_yolov8_client, Trigger.Request())
        response.success = True
        response.message = "stop"
        return response
    def shutdown(self, signum, frame):
        self.running = False

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put(bgr_image)

    def pick(self, pose_t, angle):
        waste_category = None
        if self.start_move:
            time.sleep(0.2)
            for k, v in WASTE_CLASSES.items():
                if self.current_class_name in v:
                    waste_category = k
                    break
            self.class_name = None
            self.get_logger().info('\033[1;32m%s\033[0m' % waste_category)
            self.stop_srv_callback(Trigger.Request(), Trigger.Response())
            msg = set_pose_target(pose_t, self.pick_pitch,  [-90.0, 90.0], 1.0)
            res = self.send_request(self.kinematics_client, msg)
            servo_data = res.pulse  
          
            angle = 500 + int(1000 * (angle / 240))
            # 驱动舵机
            set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            time.sleep(1)
            
            set_servo_position(self.joints_pub, 1.0, ((5, angle),))
            time.sleep(1)

            pose_t[2] -= 0.03
            msg = set_pose_target(pose_t, self.pick_pitch,  [-90.0, 90.0], 1.0)
            res = self.send_request(self.kinematics_client, msg)
            servo_data = res.pulse 

            set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            time.sleep(1)
            set_servo_position(self.joints_pub, 0.5, ((10, 600),))
            time.sleep(0.5)

            pose_t[2] += 0.1
            msg = set_pose_target(pose_t, self.pick_pitch,  [-180.0, 180.0], 1.0)
            res = self.send_request(self.kinematics_client, msg)
            servo_data = res.pulse  

            set_servo_position(self.joints_pub, 1.0, ((2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
            time.sleep(1)

            set_servo_position(self.joints_pub, 1.0, ((1, 500), (2, 610), (3, 70), (4, 140), (5, 500)))  # 设置机械臂初始位置
            time.sleep(1)
            self.start_move = False
            self.start_place = True
            threading.Thread(target=self.place, args=(waste_category,),daemon=True).start()
        else:
            time.sleep(0.01)

    def place(self, waste_category):
        
        if self.start_place:
            for k, v in POSITIONS.items():
                if waste_category in k:
                    position = v
                    break
                   
            msg = set_pose_target(position, self.pick_pitch,  [-90.0, 90.0], 1.0)
            res = self.send_request(self.kinematics_client, msg)
            if res.pulse : # 可以达到
                servo_data = res.pulse  
          
                # 驱动舵机
                set_servo_position(self.joints_pub, 1.0, ((1, servo_data[0]), ))
                time.sleep(1)
                set_servo_position(self.joints_pub, 1.0, ( (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
                time.sleep(1)
            
                set_servo_position(self.joints_pub, 0.5, ((10, 200),))
                time.sleep(1)
    
                
                set_servo_position(self.joints_pub, 1.0, ( (2, 610), (3, 70), (4, 140), (5, 500) ))  
                time.sleep(1)
                set_servo_position(self.joints_pub, 1.0, ((1, 500),))  
                time.sleep(1)

                self.start_srv_callback(Trigger.Request(), Trigger.Response())
                self.class_name = None
                self.start_place = False
                self.start_count = False
            else:
                time.sleep(0.01)
    def main(self):
        count = 0
        while self.running:
            try:
                image = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            if self.class_name is not None and not self.start_move and not self.start_count and not self.debug:
                self.count += 1
                if self.count > 20:
                    self.current_class_name = self.class_name
                    self.start_move = True
                    self.start_count = True
                    self.count = 0
            elif self.debug and self.class_name is not None:
                count += 1
                if count > 50:
                    count = 0

                    self.debug = False
            else:
                self.count = 0
                time.sleep(0.01)
            if image is not None:
                # self.fps.update()
                # image = self.fps.show_fps(image)
                cv2.imshow('image', image)
                key = cv2.waitKey(1)
                if key == ord('q') or key == 27:  # 按q或者esc退出
                    self.running = False

    def get_object_callback(self, msg):
        objects = msg.objects
        if objects == []:
            self.center = None
            self.class_name = None
        else:
            for i in objects:
                center = (int(i.box[0]), int(i.box[1]))

                self.class_name = i.class_name
                r = i.angle
                r = r % 90 # 将旋转角限制到 ±45°(limit the rotation angle to ±45°)
                angle = r - 90 if r > 45 else (r + 90 if r < -45 else r)
                projection_matrix = np.row_stack((np.column_stack((self.extristric[1], self.extristric[0])), np.array([[0, 0, 0, 1]])))
                world_pose = common.pixels_to_world([center, ], self.K, projection_matrix)[0]  # 像素坐标相对于识别区域中心的相对坐标(pixel coordinates relative to the center of the recognition area)
                world_pose[1] = -world_pose[1]
                world_pose[2] = 0.04
                world_pose = np.matmul(self.white_area_center, common.xyz_euler_to_mat(world_pose, (0, 0, 0)))  # 转换到相机相对坐标(convert to the camera relative coordinates)
                world_pose[2] = 0.04
                pose_t, _ = common.mat_to_xyz_euler(world_pose)
                pose_t[2] = 0.015
                config_data = common.get_yaml_data("/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/positions.yaml")
                offset = tuple(config_data['waste_classification']['offset'])
                scale = tuple(config_data['waste_classification']['scale'])
                for i in range(3):
                    pose_t[i] = pose_t[i] + offset[i]
                    pose_t[i] = pose_t[i] * scale[i]

                pose_t[2] += (math.sqrt(pose_t[1] ** 2 + pose_t[0] ** 2) - 0.15) / 0.20 * 0.020
                threading.Thread(target=self.pick, args=(pose_t, angle), daemon=True).start()


def main():
    node = WasteClassificationNode('waste_classification')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

