#!/usr/bin/env python3
# encoding: utf-8
# 蜂鸣器控制例程

import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import BuzzerState

class BuzzerController(Node):
    def __init__(self):
        super().__init__('buzzer_controller')
        self.pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

        #等待机械臂底层控制服务启动
        self.client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.client.wait_for_service()

    def set_buzzer(self, freq, on_time, off_time, repeat):
        msg = BuzzerState()
        msg.freq = freq
        msg.on_time = on_time
        msg.off_time = off_time
        msg.repeat = repeat
        
        # 发布消息
        self.pub.publish(msg)
        self.get_logger().info(f'Published BuzzerState: freq={msg.freq}, on_time={msg.on_time}, off_time={msg.off_time}, repeat={msg.repeat}')

def main(args=None):
    rclpy.init(args=args)
    controller = BuzzerController()

    # 发送蜂鸣器状态
    controller.set_buzzer(freq=1500, on_time=0.1, off_time=0.5, repeat=10)
    time.sleep(5)
    controller.destroy_node()  # 清理节点
    rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()

