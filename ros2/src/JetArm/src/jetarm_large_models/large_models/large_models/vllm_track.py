#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import cv2
import json
import time
import queue
import rclpy
import threading
import PIL.Image
import numpy as np
import sdk.fps as fps
from sdk import common
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String, Float32, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from rcl_interfaces.msg import SetParametersResult
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel
from large_models.track_anything import ObjectTracker
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from kinematics.kinematics_control import set_pose_target, set_joint_value_target
from servo_controller.bus_servo_control import set_servo_position
from servo_controller_msgs.msg import ServosPosition, ServoPosition

PROMPT = '''
你作为图像识别专家，你的能力是将用户发来的图片进行目标检测精准定位，并按「输出格式」进行最后结果的输出。
## 1. 理解用户指令
我会给你一句话，你需要根据我的话做出最佳决策，从做出的决策中提取「物体名称」, **object对应的name要用英文表示**, **不要输出没有提及到的物体**
## 2. 理解图片
我会给你一张图, 从这张图中找到「物体名称」对应物体的左上角和右下角的像素坐标, **不要输出没有提及到的物体**
【特别注意】： 要深刻理解物体的方位关系
## 输出格式（请仅输出以下内容，不要说任何多余的话)
{
    "object": name, 
    "xyxy": [xmin, ymin, xmax, ymax]
}
'''

display_size = [int(640*6/4), int(480*6/4)]
class VLLMTrack(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.fps = fps.FPS() # 帧率统计器(frame rate counter)
        self.image_queue = queue.Queue(maxsize=2)
        self.vllm_result = ''
        # self.vllm_result = '''json{"object":"红色方块", "xyxy":[521, 508, 637, 683]}'''
        self.running = True
        self.data = []
        self.start_track = False
        self.track = ObjectTracker()
        timer_cb_group = ReentrantCallbackGroup()
        self.client = speech.OpenAIAPI(api_key, base_url)
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(Image, 'depth_cam/rgb/image_raw', self.image_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.create_subscription(String, 'agent_process/result', self.vllm_result_callback, 1)

        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.ik_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        self.ik_client.wait_for_service()
        self.fk_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target')
        self.fk_client.wait_for_service()

        # 定义 PID 参数
        # 0.07, 0, 0.001
        self.pid_params = {
            'kp1': 0.065, 'ki1': 0.0, 'kd1': 0.001,
            'kp2': 0.00004, 'ki2': 0.0, 'kd2': 0.0,
            'kp3': 0.05, 'ki3': 0.0, 'kd3': 0.0
        }

        # 动态声明参数
        for param_name, default_value in self.pid_params.items():
            self.declare_parameter(param_name, default_value)
            self.pid_params[param_name] = self.get_parameter(param_name).value

        self.track.update_pid([self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
                      [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']],
                      [self.pid_params['kp3'], self.pid_params['ki3'], self.pid_params['kd3']])

        # 动态更新时的回调函数
        self.add_on_set_parameters_callback(self.on_parameter_update)

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def on_parameter_update(self, params):
        """参数更新回调"""
        for param in params:
            if param.name in self.pid_params.keys():
                self.pid_params[param.name] = param.value
        # self.get_logger().info(f'PID parameters updated: {self.pid_params}')
        # 更新 PID 参数
        self.track.update_pid([self.pid_params['kp1'], self.pid_params['ki1'], self.pid_params['kd1']],
                      [self.pid_params['kp2'], self.pid_params['ki2'], self.pid_params['kd2']],
                      [self.pid_params['kp3'], self.pid_params['ki3'], self.pid_params['kd3']])

        return SetParametersResult(successful=True)

    def create_update_callback(self, param_name):
        """生成动态更新回调"""
        def update_param(msg):
            new_value = msg.data
            self.pid_params[param_name] = new_value
            self.set_parameters([Parameter(param_name, Parameter.Type.DOUBLE, new_value)])
            self.get_logger().info(f'Updated {param_name}: {new_value}')
            # 更新 PID 参数

        return update_param

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()
        
        msg = SetModel.Request()
        msg.model_type = 'vllm'
        msg.model = stepfun_vllm_model
        msg.api_key = stepfun_api_key
        msg.base_url = stepfun_base_url
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)
        
        z = 0.15
        msg = set_pose_target([0.137, 0.0, z], 30, [-90.0, 90.0], 1.0)
        res = self.send_request(self.ik_client, msg)
        if res.pulse:
            servo_data = res.pulse
            # self.get_logger().info(str(servo_data))
            set_servo_position(self.joints_pub, 1.5, ((10, 300), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            time.sleep(1.8)
            self.track.set_init_param(servo_data[3], servo_data[0], z)
        speech.play_audio(start_audio_path)
        threading.Thread(target=self.process, daemon=True).start()
        threading.Thread(target=self.track_thread, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def vllm_result_callback(self, msg):
        self.vllm_result = msg.data

    def play_audio_finish_callback(self, msg):
        if msg.data:
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)

    def process(self):
        box = ''
        cv2.namedWindow('image', 0)
        cv2.moveWindow('image', 1920 - display_size[0], 0)
        while self.running:
            image = self.image_queue.get(block=True)
            if self.vllm_result:
                try:
                    self.vllm_result = json.loads(self.vllm_result)
                    box = self.vllm_result['xyxy']
                    box = self.client.data_process(box, 640, 480)
                    box = [box[0], box[1], box[2] - box[0], box[3] - box[1]]
                    self.get_logger().info('box: %s' % str(box))
                    self.track.set_track_target(box, image)
                    self.start_track = True
                    speech.play_audio(start_track_audio_path, block=False)
                except (ValueError, TypeError):
                    self.start_track = False
                    msg = String()
                    msg.data = self.vllm_result
                    self.tts_text_pub.publish(msg)
                    # speech.play_audio(track_fail_audio_path, block=False)
                    # self.get_logger().info(e)
                self.vllm_result = ''
                msg = SetBool.Request()
                msg.data = True
                self.send_request(self.awake_client, msg)
            if self.start_track:
                self.data = self.track.track(image)
            self.fps.update()
            self.fps.show_fps(image)
            cv2.imshow('image', cv2.resize(image, (display_size[0], display_size[1])))
            cv2.waitKey(1)

        cv2.destroyAllWindows()

    def track_thread(self):
        while self.running:
            if self.data:
                y, z, joint4 = self.data[0], self.data[1], self.data[2]
                if z > 0.23 or joint4 >= 391:
                    msg = set_pose_target([0.18, 0.0, z], 0.0, [-180.0, 180.0], 1.0)
                    res = self.send_request(self.ik_client, msg)
                    if res.pulse:
                        servo_data = res.pulse
                        set_servo_position(self.joints_pub, 0.0, (
                        (10, 500), (5, 500), (4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]),
                        (1, int(y))))
                    else:
                        set_servo_position(self.joints_pub, 0.02, ((1, int(y)),))
                else:
                    set_servo_position(self.joints_pub, 0.0, ((1, y), (4, joint4)))
                self.data = []
            else:
                time.sleep(0.01)

    def image_callback(self, ros_image):
        rgb_image = np.ndarray(shape=(ros_image.height, ros_image.width, 3), dtype=np.uint8,
                               buffer=ros_image.data)  # 原始 RGB 画面
        image = cv2.cvtColor(rgb_image, cv2.COLOR_RGB2BGR)
        if self.image_queue.full():
            # # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
            # # 将图像放入队列
        self.image_queue.put(image)

def main():
    node = VLLMTrack('vllm_track')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()

