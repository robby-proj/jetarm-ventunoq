#!/usr/bin/env python3
# encoding: utf-8
# LED控制例程
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import LedState

class LedController(Node):
    def __init__(self):
        super().__init__('led_controller')
        self.publisher = self.create_publisher(LedState, '/ros_robot_controller/set_led', 10)
        
        #等待机械臂底层控制服务启动
        self.client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.client.wait_for_service()

        # 发布消息
        self.publish_led_state()

    def publish_led_state(self):
        msg = LedState()
        msg.id = 2
        msg.on_time = 0.1
        msg.off_time = 0.5
        msg.repeat = 10

        self.publisher.publish(msg)
        self.get_logger().info(f'Published LED State: id={msg.id}, on_time={msg.on_time}, off_time={msg.off_time}, repeat={msg.repeat}')

def main(args=None):
    rclpy.init(args=args)
    led_controller = LedController()

    rclpy.spin_once(led_controller)  # 发布一次
    rclpy.shutdown()

if __name__ == '__main__':
    main()

