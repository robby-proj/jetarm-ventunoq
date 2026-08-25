#!/usr/bin/env python3
# encoding: utf-8
# 逆运动学控制例程
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.inverse_kinematics import get_ik
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class IkDemo(Node):
    def __init__(self):
        super().__init__('ik_demo_node')

        self.coordinate = [0.35, 0.0, 0.24]
        self.servo_list = []
        self.duration = 1.0



        # 发布总线舵机话题
        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        #等待机械臂底层控制服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        # 获取逆解
        self.get_logger().info(f"末端坐标：{self.coordinate}")
        res = get_ik(self.coordinate, 0, [-180, 180])
        if res != []:
            for i in range(len(res)):
                self.get_logger().info(f'rpy{i + 1}: {res[i][1]}')
                pulse = transform.angle2pulse(res[i][0])
                self.servo_list = pulse[0]
                for j in range(len(pulse)):
                    self.get_logger().info(f'舵机脉宽值{j + 1}: {pulse[j]}')

            self.set_servo_position(self.servos_pub, self.duration, self.servo_list)#发布舵机信息，驱动机械臂
        else:
            self.get_logger().info("没有逆运动学解，请检查末端坐标")

    def set_servo_position(self, pub, duration, positions):
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for i in range(1, 6):
            position = ServoPosition()
            position.id = i
            position.position = float(positions[i-1])
            position_list.append(position)
        msg.position = position_list
        msg.position_unit = "pulse"
        pub.publish(msg)
    


def main(args=None):
    rclpy.init(args=args)  # 初始化 rclpy
    ik_demo_node = IkDemo()  # 创建节点
    rclpy.spin(ik_demo_node)  # 保持节点运行
    fk_demo_node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 rclpy

if __name__ == '__main__':
    main()

