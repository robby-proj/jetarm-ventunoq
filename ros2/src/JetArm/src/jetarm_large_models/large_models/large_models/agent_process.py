#!/usr/bin/env python3
# encoding: utf-8
# @Author: Aiden
# @Date: 2024/11/18
import json
import requests
import queue
import rclpy
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import String
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger, SetBool, Empty

from speech import speech
from large_models.config import *
from large_models_msgs.srv import SetString, SetModel, SetContent

class AgentProcess(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.declare_parameter('camera_topic', 'depth_cam/rgb/image_raw')
        camera_topic = self.get_parameter('camera_topic').value

        self.prompt = ''
        self.model = llm_model 
        self.chat_text = ''
        self.start_record_chat = False
        self.bridge = CvBridge()
        self.image_queue = queue.Queue(maxsize=2)
        self.client = speech.OpenAIAPI(api_key, base_url)
        
        self.result_pub = self.create_publisher(String, '~/result', 1)
        self.create_subscription(String, 'vocal_detect/asr_result', self.asr_callback, 1)
        self.create_subscription(Image, camera_topic, self.image_callback, 1)  # 摄像头订阅(subscribe to the camera)
        self.create_service(SetModel, '~/set_model', self.set_model_srv)
        self.create_service(SetString, '~/set_prompt', self.set_prompt_srv)
        self.create_service(SetContent, '~/set_llm_content', self.set_llm_content_srv)
        self.create_service(SetContent, '~/set_vllm_content', self.set_vllm_content_srv)

        self.create_service(SetBool, '~/record_chat', self.record_chat)
        self.create_service(Trigger, '~/get_chat', self.get_chat)
        self.create_service(Empty, '~/clear_chat', self.clear_chat)

        self.create_service(Empty, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        return response

    def record_chat(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'record chat')
        self.start_record_chat = request.data
        response.success = True
        return response

    def get_chat(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'get chat')
        response.message = self.chat_text.rstrip(",")
        response.success = True
        return response

    def clear_chat(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % 'clear chat')
        self.chat_text = ''
        self.record_chat = False
        return response

    def asr_callback(self, msg):
        # self.get_logger().info(msg.data)
        # 将识别结果传给智能体让他来回答
        if msg.data != '':
            self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
            if self.start_record_chat:
                self.chat_text += msg.data + ','
                self.get_logger().info('\033[1;32m%s\033[0m' % 'record chat:' + self.chat_text)
            res = ''
            if self.model_type == 'llm':
                res = self.client.llm(msg.data, self.prompt, model=self.model)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish llm result:' + str(res))
            elif self.model_type == 'vllm':
                image = self.image_queue.get(block=True)
                res = self.client.vllm(msg.data, image, prompt=self.prompt, model=self.model)
                self.get_logger().info('\033[1;32m%s\033[0m' % 'publish vllm result:' + str(res))
            msg = String()
            msg.data = res
            self.result_pub.publish(msg)
        else:
            self.get_logger().info('\033[1;32m%s\033[0m' % 'asr result none')

    def image_callback(self, ros_image):
        cv_image = self.bridge.imgmsg_to_cv2(ros_image, "bgr8")
        bgr_image = np.array(cv_image, dtype=np.uint8)
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像(if the queue is full, discard the oldest image)
            self.image_queue.get()
        # 将图像放入队列(put the image into the queue)
        self.image_queue.put(bgr_image)

    def set_model_srv(self, request, response):
        # 设置调用哪个模型
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set model')
        self.model = request.model
        self.model_type = request.model_type
        self.client = speech.OpenAIAPI(request.api_key, request.base_url)
        response.success = True
        return response

    def set_prompt_srv(self, request, response):
        # 设置大模型的prompt
        self.get_logger().info('\033[1;32m%s\033[0m' % 'set prompt')
        self.prompt = request.data
        response.success = True
        return response

    def set_llm_content_srv(self, request, response):
        # 输入文本传给智能体让他来回答
        self.get_logger().info('\033[1;32m%s\033[0m' % 'thinking...')
        client = speech.OpenAIAPI(request.api_key, request.base_url)
        response.message = client.llm(request.query, request.prompt, model=request.model) 
        response.success = True
        return response

    def set_vllm_content_srv(self, request, response):
        # 输入提示词和文本，视觉智能体返回回答
        image = self.image_queue.get(block=True)
        client = speech.OpenAIAPI(request.api_key, request.base_url)
        res = client.vllm(request.query, image, prompt=request.prompt, model=request.model) 
        response.message = res
        response.success = True
        return response

def main():
    node = AgentProcess('agent_process')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('shutdown')
    finally:
        rclpy.shutdown() 

if __name__ == "__main__":
    main()
