#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from chassis import mecanum
from chassis_msgs.msg import Mecanum
import threading


class ChassisController(Node):
    def __init__(self, name):
        super().__init__(name)
        self.wheel_diameter_tank = 0.052
        self.wheel_diameter_mecanum = 0.097
        self.chassis_type = os.environ['CHASSIS_TYPE']
        self.chassis = mecanum.MecanumChassis()
        # self.mecanum_pub = self.create_publisher(Mecanum, '/chassis_controller/status',  1)
        self.create_subscription(Mecanum, '/chassis_controller/command', self.chassis_controller, 10)
    def chassis_controller(self, msg):
        self.get_logger().info('\033[1;32m%s\033[0m' % str(msg))

        if self.chassis_type == 'Mecanum':
            self.chassis.set_velocity(msg.velocity, msg.direction, msg.angular_rate)
        elif self.chassis_type == 'Tank':
            self.chassis.set_velocity((self.wheel_diameter_mecanum/self.wheel_diameter_tank)*msg.velocity, msg.direction, msg.angular_rate)
        else:
            pass
 
def main():
    
    rclpy.init()
    node = ChassisController('chassis_controller')
    rclpy.spin(node)
    camera_node.destroy_node()
    rclpy.shutdown()     


if __name__ == '__main__':
    main()
