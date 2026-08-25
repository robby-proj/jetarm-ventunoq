#!/usr/bin/env python3
# encoding: utf-8
import rclpy
import time
import threading
from rclpy.node import Node
from std_srvs.srv import Trigger
from rclpy.executors import MultiThreadedExecutor
from ros_robot_controller_msgs.msg import BuzzerState
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from ros_robot_controller_msgs.msg import  GetBusServoCmd
from rclpy.callback_groups import ReentrantCallbackGroup
from ros_robot_controller_msgs.srv import GetBusServoState
from ros_robot_controller import ros_robot_controller_node

class CheckServoConnection(Node):
    
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)

        self.buzzer_pub = self.create_publisher(BuzzerState, 'ros_robot_controller/set_buzzer', 1)
        timer_cb_group = ReentrantCallbackGroup()
        self.bus_servo_state_client = self.create_client(GetBusServoState, 'ros_robot_controller/bus_servo/get_state', callback_group=timer_cb_group)
        self.bus_servo_state_client.wait_for_service()
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.servo_ids_to_check = [1, 2, 3, 4, 5, 10]
        self.timer = self.create_timer(0.0, self.check_servo_connections, callback_group=timer_cb_group)

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done(): 
                if future.result():
                    return future.result()


    def check_servo_connections(self):
        self.timer.cancel()
        request = GetBusServoState.Request()
        cmd = GetBusServoCmd()
        for i in self.servo_ids_to_check:
            try:
                cmd.id = int(i)
                cmd.get_id = int(1)
                request.cmd = [cmd] 
                res = self.send_request(self.bus_servo_state_client, request)

                if res is not None and res.state and res.state[0].present_id:
                    servo_id = res.state[0].present_id[0]
                    if servo_id == i:
                        self.get_logger().info(f"Detected serial servo ID: {i}")
                    else:
                        self.get_logger().error(f'Detected serial servo ID: {i}')
                else:
                    self.handle_servo_not_found(i)

            except Exception as e:
                self.get_logger().error(str(e))
  
 
    def handle_servo_not_found(self, ID):
        while True:
            try:
                msg = BuzzerState()
                msg.freq = 1900
                msg.on_time = 0.1
                msg.off_time = 0.15
                msg.repeat = 1
                self.buzzer_pub.publish(msg)
                time.sleep(0.5)
                self.get_logger().error(f"Could not detect serial servo ID: {ID} on the bus!!! Node is shutting down.")  
            except Exception as e:
                self.get_logger().error(str(e))
                break  
 
 

def main():
    node = CheckServoConnection('check_servo_connection')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
if __name__ == '__main__':
    main()

    
