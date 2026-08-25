#!/usr/bin/env python3
# encoding: utf-8
# 舵机控制例程
import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ros_robot_controller_msgs.msg import ServoPosition, ServosPosition

class ServoController(Node):
    def __init__(self):
        super().__init__('servo_control_demo')
        self.pub = self.create_publisher(ServosPosition, '/ros_robot_controller/bus_servo/set_position', 1)

        #等待机械臂底层控制服务启动
        self.client = self.create_client(Trigger, '/ros_robot_controller/init_finish')
        self.client.wait_for_service()

    def set_servo_position(self, duration, positions):
        msg = ServosPosition()
        msg.duration = float(duration)
        position_list = []
        for i in positions:
            position = ServoPosition()
            position.id = i[0]
            position.position = int(i[1])
            position_list.append(position)
        msg.position = position_list
        self.pub.publish(msg)
        for pos in position_list:
            self.get_logger().info(f'duration={msg.duration}, id={pos.id}, position={pos.position}')

def main(args=None):
    rclpy.init(args=args)
    controller = ServoController()

    try:
        while rclpy.ok():
            controller.set_servo_position(0.5, ((4, 200),))  # 设置舵机 ID 4 到位置 200
            time.sleep(0.5)  # 等待 0.5 秒
            controller.set_servo_position(0.5, ((4, 500),))  # 设置舵机 ID 4 到位置 500
            time.sleep(0.5)  # 等待 0.5 秒
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()  # 清理节点
        rclpy.shutdown()  # 关闭 ROS 2

if __name__ == '__main__':
    main()

