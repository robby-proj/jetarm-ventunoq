#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/12/03
import time
import json
import rclpy
import threading
from config import *
from sdk import common
from speech import speech
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger, SetBool, Empty
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from large_models.config import *
from large_models_msgs.srv import SetModel, SetString
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from servo_controller.bus_servo_control import set_servo_position
from kinematics.kinematics_control import set_joint_value_target, set_pose_target

PROMPT = '''
#角色任务
你是一款智能机械臂，需要根据输入的内容，生成对应的指令。

##要求与限制
1.根据输入的动作内容，在动作函数库中找到对应的指令，并输出对应的指令。
2.为每个动作序列编织一句精炼（10至30字）、风趣且变化无穷的反馈信息，让交流过程妙趣横生。
3.上为z轴正，左为y轴负，前为x轴正，单位米
4.舵机编号为1,2,3,4,5,10，转动范围为[-120, 120]度，如果超过了范围action就返回[]
5.初始位置[0, 48, -72, -96, 0, -30]
6.直接输出json结果，不要分析，不要输出多余内容。
7.格式：{'action':['xx', “xx”], 'response':'xx'}

##结构要求##：
- `"action"`键下承载一个按执行顺序排列的函数名称字符串数组，当找不到对应动作函数时action输出[]。 
- `"response"`键则配以精心构思的简短回复，完美贴合上述字数与风格要求。 
- **特殊处理**：如果没有指定舵机运行时间就固定为1秒， 如果两个动作间没有延时或者停顿就把他们合一起

## 动作函数库
- 控制舵机在指定时间转到指定位置：set_position(1, ((1, -120), (2, 90)))
- 控制机械臂往指定方向移动一段距离：move(x, y, z)
- 延时指定时间：time.sleep(1)

## 任务示例
输入：1号舵机，2号舵机在2秒内分别转到-90度和90度
输出：{'action':['set_position(2, [[1, -90], [2, 90]])'], 'response':'收到，马上执行'}
输入：1号舵机在1秒内转到45度
输出：{'action':['set_position(1, [[1, 45]])'], 'response':'好的'}
输入：6号舵机转到15度
输出：{'action':['set_position(1, [[6, 15]])'], 'response':'好的，6号舵机转到15度'}
输入：3号舵机转到11度， 4号舵机转到0度，停2s
输出：{'action':['set_position(1, [[3, 11], [4, 0]])', 'time.sleep(2)'], 'response':'好嘞'}
输入：1号舵机转到20度， 3号舵机转到45度，停1s, 1号舵机转到50度， 2号舵机转到0度
输出：{'action':['set_position(1, [[1, 20], [3, 45]])', 'time.sleep(1)','set_position(1, [[2, 0]])'], 'response':'好嘞'}

'''

class LLMControlServo(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.action = []
        self.interrupt = False
        self.llm_result = ''
        # self.llm_result = '{\'action\':[\'set_position(1, [[10, -20]])\'], \'response\':\'好嘞\'}'
        # self.llm_result = '{\"action\": [\"move(0, 0, -0.05)\"], \"response\": \"坐稳了，准备下移5cm！\"}'
        self.running = True
        self.id_dict = {'1': 0, "2": 1, "3": 2, "4": 3, "5": 4, "10": 5}
        self.current_pulse = [500, 700, 115, 100, 500, 575]
        self.current_position = [0.1, 0, 0.226]

        timer_cb_group = ReentrantCallbackGroup()
        self.tts_text_pub = self.create_publisher(String, 'tts_node/tts_text', 1)
        self.create_subscription(String, 'agent_process/result', self.llm_result_callback, 1)
        self.create_subscription(Bool, 'tts_node/play_finish', self.play_audio_finish_callback, 1, callback_group=timer_cb_group)
        self.awake_client = self.create_client(SetBool, 'vocal_detect/enable_wakeup')
        self.awake_client.wait_for_service()
        self.create_subscription(Bool, 'vocal_detect/wakeup', self.wakeup_callback, 1)

        self.set_model_client = self.create_client(SetModel, 'agent_process/set_model')
        self.set_model_client.wait_for_service()
        self.set_prompt_client = self.create_client(SetString, 'agent_process/set_prompt')
        self.set_prompt_client.wait_for_service()
        self.set_pose_target_client = self.create_client(SetRobotPose, 'kinematics/set_pose_target')
        self.set_pose_target_client.wait_for_service()
        self.set_joint_value_target_client = self.create_client(SetJointValue, 'kinematics/set_joint_value_target')
        self.set_joint_value_target_client.wait_for_service()
        self.joints_pub = self.create_publisher(ServosPosition, 'servo_controller', 1) # 舵机控制

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def get_node_state(self, request, response):
        return response

    def init_process(self):
        self.timer.cancel()

        msg = SetModel.Request()
        msg.model = llm_model
        msg.model_type = 'llm'
        msg.api_key = api_key 
        msg.base_url = base_url
        self.send_request(self.set_model_client, msg)

        msg = SetString.Request()
        msg.data = PROMPT
        self.send_request(self.set_prompt_client, msg)
        # -92 
        self.set_position(1.5, [[1, 0], [2, 48], [3, -72], [4, -96], [5, 0], [10, -30]])
        msg = set_joint_value_target(self.current_pulse[:-1])
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t, pose_r = endpoint.pose.position, endpoint.pose.orientation
        mat = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        position, rpy = common.mat_to_xyz_euler(mat)
        self.current_position = [position, rpy]

        speech.play_audio(start_audio_path) 
        threading.Thread(target=self.process, daemon=True).start()
        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def wakeup_callback(self, msg):
        if msg.data and self.llm_result:
            self.interrupt = True

    def llm_result_callback(self, msg):
        self.llm_result = msg.data

    def set_position(self, duration, positions):
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for i in positions:
            position = ServoPosition()
            position.id = i[0]
            position.position = float(i[1])
            position_list.append(position)
            if i[0] != 10: 
                self.current_pulse[self.id_dict[str(i[0])]] = 500 + float(i[1])/240*1000
            else:
                self.current_pulse[self.id_dict[str(i[0])]] = 700 + float(i[1])/240*1000
        msg.position = position_list
        msg.position_unit = "deg"
        self.joints_pub.publish(msg)
        # self.get_logger().info(f'{self.current_pulse}') 
        msg = set_joint_value_target(self.current_pulse[:-1])
        endpoint = self.send_request(self.set_joint_value_target_client, msg)
        pose_t, pose_r = endpoint.pose.position, endpoint.pose.orientation
        mat = common.xyz_quat_to_mat([pose_t.x, pose_t.y, pose_t.z], [pose_r.w, pose_r.x, pose_r.y, pose_r.z])
        position, rpy = common.mat_to_xyz_euler(mat)
        self.current_position = [position, rpy]
        # self.get_logger().info(f'{self.current_position}')

    def move(self, x, y, z):
        self.current_position[0][0] += x
        self.current_position[0][1] += y
        self.current_position[0][2] += z
        msg = set_pose_target(self.current_position[0], self.current_position[1][1], [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1, ((4, servo_data[3]), (3, servo_data[2]), (2, servo_data[1]), (1, servo_data[0])))
            self.current_pulse[:-1] = servo_data

    def play_audio_finish_callback(self, msg):
        if msg.data:
            msg = SetBool.Request()
            msg.data = True
            self.send_request(self.awake_client, msg)

    def process(self):
        while self.running:
            if self.llm_result:
                msg = String()
                if 'action' in self.llm_result:  # 如果有对应的行为返回那么就提取处理
                    result = eval(self.llm_result[self.llm_result.find('{'):self.llm_result.find('}') + 1])
                    # self.get_logger().info(str(result))
                    action_list = []
                    if 'action' in result:
                        action_list = result['action']
                    if 'response' in result:
                        response = result['response']
                    msg.data = response
                    self.tts_text_pub.publish(msg)
                    for i in action_list:
                        if self.interrupt:
                            self.get_logger().info('interrupt')
                            break
                        if 'set_position' in i:
                            eval('self.' + i)
                        elif 'move' in i:
                            eval('self.' + i)
                        else:
                            eval(i)
                    self.interrupt = False
                else:  # 没有对应的行为，只回答
                    response = self.llm_result
                    msg.data = response
                    self.tts_text_pub.publish(msg)
                self.llm_result = ''
            else:
                time.sleep(0.01)
        rclpy.shutdown()

def main():
    node = LLMControlServo('llm_control_servo')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()
