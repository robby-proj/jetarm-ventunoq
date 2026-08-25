import time
import os
import math
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import numpy as np
from math import radians
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import BuzzerState
from servo_controller import bus_servo_control
from servo_controller_msgs.msg import ServosPosition, ServoPosition
from kinematics_msgs.srv import SetRobotPose, SetJointValue
from kinematics.kinematics_control import set_pose_target


class PathPlanning(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        
        self.start = True
        self.servos_list = [500, 610, 70, 140, 500, 200]

        self.servos_pub = self.create_publisher(ServosPosition, 'servo_controller', 1)# 舵机控制

        #等待服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.kinematics_client.wait_for_service()

        bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, self.servos_list[0]), (2, self.servos_list[1]), (3, self.servos_list[2]), (4, self.servos_list[3]), (5, self.servos_list[4]), (10, self.servos_list[5])))  # 设置机械臂初始位置
        self.run()

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done():
                try:
                    response = future.result()
                    return response
                except Exception as e:
                    self.get_logger().error('Service call failed: {}'.format(e))
                    return None
            rclpy.spin_once(self)  # 允许节点处理回调函数
  

    def move_0(self, x, y, z, pitch, t=1000):
        # 是否可以规划机械臂运动到设置的坐标
        msg = set_pose_target([x, y, z], pitch, [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)

        if res.pulse:# 可以达到
            servo_data = res.pulse
            self.get_logger().info(f"舵机角度: {list(res.pulse)}")

            # 驱动舵机
            bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, servo_data[0]),))
            time.sleep(1.2)

        else:
            self.start = False
            self.get_logger().info("无法运行到此位置")

    def move_1(self, x, y, z, pitch, t=1000):
        # 是否可以规划机械臂运动到设置的坐标
        msg = set_pose_target([x, y, z], pitch, [-90.0, 90.0], 1.0)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse: # 可以达到
            servo_data = res.pulse  
            self.get_logger().info(f"舵机角度: {list(res.pulse)}")
          
            # 驱动舵机
            bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3]), (5, servo_data[4])))

        else:
            self.start = False
            self.get_logger().info("无法运行到此位置")

    def run(self):
        while self.start:
            # 运行路径规划程序
            self.get_logger().info("云台运动")
            self.move_0(0.2, -0.1, 0.05, 90)
            time.sleep(2)
            self.get_logger().info("动作1")
            self.move_1(0.2, -0.1, 0.05, 90)
            time.sleep(2)
            self.get_logger().info("动作2")
            self.move_1(0.2, -0.1, 0.005, 90)
            time.sleep(3)
            self.start = False  # 停止循环
            self.get_logger().info("停止运动")

        bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, self.servos_list[0]), (2, self.servos_list[1]), (3, self.servos_list[2]), (4, self.servos_list[3]), (5, self.servos_list[4]), (10, self.servos_list[5])))  # 设置机械臂初始位置
        time.sleep(1)


def main():
    node = PathPlanning('path_planning')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

