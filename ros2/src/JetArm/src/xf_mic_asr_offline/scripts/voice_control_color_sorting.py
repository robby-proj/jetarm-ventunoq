#!/usr/bin/env python3
# encoding: utf-8
# 语音开启颜色分拣
import os
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from interfaces.srv import SetStringBool
from std_srvs.srv import Trigger, SetBool
from xf_mic_asr_offline import voice_play
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup

class VoiceControlColorSortingNode(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        
        self.running = True
        self.language = os.environ['ASR_LANGUAGE']
        timer_cb_group = ReentrantCallbackGroup()
        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.create_subscription(String, '/asr_node/voice_words', self.words_callback, 1, callback_group=timer_cb_group)

        self.client = self.create_client(Trigger, '/asr_node/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/object_sorting/init_finish')
        self.client.wait_for_service()   
        self.set_target_client = self.create_client(SetStringBool, '/object_sorting/set_target', callback_group=timer_cb_group)
        self.set_target_client.wait_for_service()

        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)

    def init_process(self):
        self.timer.cancel()
        self.play('running')
        self.get_logger().info('\033[1;32m%s\033[0m' %'唤醒口令: 小幻小幻(Wake up word: hello hiwonder)')
        self.get_logger().info('\033[1;32m%s\033[0m' %'唤醒后15秒内可以不用再唤醒(No need to wake up within 15 seconds after waking up)')
        self.get_logger().info('\033[1;32m%s\033[0m' %'控制指令: 分拣红色 分拣绿色 分拣蓝色 关闭颜色分拣(Voice command: sorting red/green/blue object stop color sorting)')

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')


    def get_node_state(self, request, response):
        response.success = True
        return response

    def play(self, name):
        voice_play.play(name, language=self.language)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def words_callback(self, msg):
        words = json.dumps(msg.data, ensure_ascii=False)[1:-1]
        if self.language == 'Chinese':
            words = words.replace(' ', '')

        if words is not None and words not in ['唤醒成功(wake-up-success)', '休眠(Sleep)', '失败5次(Fail-5-times)',
                                               '失败10次(Fail-10-times']:
            if words == '分拣红色' or words == 'sorting red object':
                self.play('start_sort_red')
                req = SetStringBool.Request()
                req.data_bool = True
                req.data_str = 'red'
                res = self.send_request(self.set_target_client, req)

            elif words == '分拣绿色' or words == 'sorting green object':
                self.play('start_sort_green')
                req = SetStringBool.Request()
                req.data_bool = True
                req.data_str = 'green'
                res = self.send_request(self.set_target_client, req)

            elif words == '分拣蓝色' or words == 'sorting blue object':
                self.play('start_sort_blue')
                req = SetStringBool.Request()
                req.data_bool = True
                req.data_str = 'blue'
                res = self.send_request(self.set_target_client, req)

            elif words == '关闭颜色分拣' or words == 'stop color sorting':
                target_list = ["red", "green", "blue"]
                req = SetStringBool.Request()
                req.data_bool = False
                for i in target_list:
                    req.data_str = i
                    res = SetBool.Response()
                    res = self.send_request(self.set_target_client, req)
                self.play('stop_sort')

        elif words == '唤醒成功(wake-up-success)':
            self.play('awake')
        elif words == '休眠(Sleep)':
            msg = BuzzerState()
            msg.freq = 1900
            msg.on_time = 0.05
            msg.off_time = 0.01
            msg.repeat = 1
            self.buzzer_pub.publish(msg)

def main():
    node = VoiceControlColorSortingNode('voice_control_color_sorting')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
 
if __name__ == "__main__":
    main()

