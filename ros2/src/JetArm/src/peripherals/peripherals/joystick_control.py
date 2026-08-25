#!/usr/bin/env python3
"""
ps2 手柄机体遥控， mode为0为单舵机模式， mode为1为坐标模式(ps2 joystick remote control, when the mode is 0, it is the single servo mode; when the mode is 1, it is the coordinate mode)
"""

import os
import math
import time
import rclpy
import numpy as np
import pygame as pg
from enum import Enum
from math import radians
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger
from chassis_msgs.msg import Mecanum
from sdk import  common, buzzer, misc
import kinematics.transform as transform
from rclpy.executors import MultiThreadedExecutor
from servo_controller import bus_servo_control, actions
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import SetRobotPose, GetRobotPose
from servo_controller_msgs.msg import ServosPosition, ServoPosition, ServoStateList
from ros_robot_controller_msgs.msg import BuzzerState, GetBusServoCmd, MotorsState, MotorState
from chassis import mecanum 
from stepper import stepper as Stepper

os.environ["SDL_VIDEODRIVER"] = "dummy"  # For use PyGame without opening a visible display
pg.display.init()

BIG_STEP = 0.1
BIG_ROTATE = math.radians(30)
SMALL_STEP = 0.06
SMALL_ROTATE = math.radians(15)

AXES_MAP = 'ly', 'lx', 'ry', 'rx'
# BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', '', 'hat_xd', 'hat_yr', 'hat_yl', '', 'laxis_r','laxis_l', 'laxis_d', 'laxis_u', 'raxis_r',  'raxis_l', 'raxis_d', 'raxis_u', 'hat_xu'
BUTTON_MAP = 'cross', 'circle', '', 'square', 'triangle', '', 'l1', 'r1', 'l2', 'r2', 'select', 'start', '', 'l3', 'r3', 'hat_xu', 'hat_xd', 'hat_yr', 'hat_yl', 'laxis_l', 'laxis_r', 'laxis_u', 'laxis_d', 'raxis_l', 'raxis_r', 'raxis_u', 'raxis_d', 'hat_xu'



class ButtonState(Enum):
    Normal = 0
    Pressed = 1
    Holding = 2
    Released = 3


class JoystickController(Node):
    def __init__(self, name):
        rclpy.init()
        super().__init__(name)
        self.count = 0
        self.joy = None
        self.mode = 0
        self.pitch = None
        self.min_value = 0.1
        self.chassis_type = os.environ['CHASSIS_TYPE']
        self.buzzer_pub = buzzer.BuzzerController()
        self.current_servo_position = None
        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)
        self.chassis_pub = self.create_publisher(Mecanum, '/chassis_controller/command', 1)
        self.motor_pub = self.create_publisher(MotorsState, '/ros_robot_controller/set_motor', 1)
        self.servo_states_sub = self.create_subscription(ServoStateList, '/controller_manager/servo_states', self.servo_states_callback, 10)

        #等待服务启动
        timer_cb_group = ReentrantCallbackGroup()
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose', callback_group=timer_cb_group)
        self.get_current_pose_client.wait_for_service()

        self.client = self.create_client(Trigger, '/controller_manager/init_finish',  callback_group=timer_cb_group)
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish', callback_group=timer_cb_group)
        self.client.wait_for_service()
        self.kinematics_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target', callback_group=timer_cb_group)
        self.kinematics_client.wait_for_service()
        self.last_axes = dict(zip(AXES_MAP, [0.0, ] * len(AXES_MAP)))
        self.last_buttons = dict(zip(BUTTON_MAP, [0.0, ] * len(BUTTON_MAP)))
        self.update_timer = self.create_timer(0.05, self.joy_callback)
        bus_servo_control.set_servo_position(self.servos_pub, 1.0, ((1, 500), (2, 560), (3, 130), (4, 115), (5, 500), (10, 200)))
        if self.chassis_type == 'Mecanum' or self.chassis_type == 'Tank':
            self.Mecanum = mecanum.MecanumChassis()
        elif self.chassis_type == 'Slide_Rails':
            self.Stepper = Stepper.Stepper(7)
            self.Stepper.set_mode(self.Stepper.EN) 


    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()
       

    def servo_states_callback(self,msg):
        servo_positions = []
        for i in msg.servo_state:
            servo_positions.append(i.position)
        self.current_servo_position = np.array(servo_positions)

    def set_slideway(self,pulse):     
        self.Stepper.goto(pulse)
        time.sleep(0.1)

    def relative_move(self, x, y, z, pitch):

        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request())
        pose = endpoint.pose
        x += pose.position.x
        y += pose.position.y
        z += pose.position.z
        rpy = transform.qua2rpy(endpoint.pose.orientation)
        self.pitch = pitch + rpy[1]
        msg = set_pose_target([x, y, z], self.pitch, pitch_range=[-50, 50], resolution=1)
        res = self.send_request(self.kinematics_client, msg)
        if res.pulse : # 可以达到
            servo_data = res.pulse  
          
            # # 驱动舵机
            bus_servo_control.set_servo_position(self.servos_pub, 0.02, ((1, servo_data[0]), (2, servo_data[1]), (3, servo_data[2]), (4, servo_data[3])))
   

    def l1_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((3, int(self.current_servo_position[2] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, 0.005, 0)

    def l2_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((3, int(self.current_servo_position[2]-10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, -0.005, 0)


    def r1_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((4, int(self.current_servo_position[3] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, 0, -1)


    def r2_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((4, int(self.current_servo_position[3] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0, 0, 1)

    def cross_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((10, int(self.current_servo_position[5] - 10)), ))

    def triangle_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((10, int(self.current_servo_position[5] + 10)), ))

    def circle_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.010, ((5, int(self.current_servo_position[4] + 10)), ))

    def square_callback(self, new_state):
        if (self.mode == 0 or self.mode == 1) and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((5, int(self.current_servo_position[4] - 10)), ))

    def raxis_u_callback(self, axes):

        pass

    def raxis_d_callback(self, axes):

        pass

    def raxis_l_callback(self, axes):

        pass

    def raxis_r_callback(self, axes):

        pass

    def laxis_u_callback(self, axes):

        pass

    def laxis_d_callback(self, axes):

        pass

    def laxis_l_callback(self, axes):
   
        pass

    def laxis_r_callback(self, axes):

        pass
      



    def hat_xu_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((2, int(self.current_servo_position[1] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0.005, 0, 0, 0)

    def hat_xd_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((2, int(self.current_servo_position[1] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(-0.005, 0, 0, 0)

    def hat_yl_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((1, int(self.current_servo_position[0] - 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, -0.005, 0, 0)

    def hat_yr_callback(self, new_state):
        if self.mode == 0 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            bus_servo_control.set_servo_position(self.servos_pub, 0.050, ((1, int(self.current_servo_position[0] + 10)), ))
        if self.mode == 1 and (new_state == ButtonState.Pressed or new_state == ButtonState.Holding):
            self.relative_move(0, 0.005, 0, 0)

    def start_callback(self, new_state):
        if new_state == ButtonState.Pressed and self.last_buttons['select']:
            if self.mode == 1:
                self.mode = 0
                self.buzzer_pub.set_buzzer(1000, 0.1, 0.3, 1)
            elif self.mode == 0:
                self.mode = 1
                self.buzzer_pub.set_buzzer(1000, 0.1, 0.5, 2)
        if new_state == ButtonState.Pressed and not self.last_buttons['select']:
            actions.go_home(self.servos_pub, 1.0)
            

    def axes_callback(self, axes):
        if abs(axes['lx']) < self.min_value:
            axes['lx'] = 0
        if abs(axes['ly']) < self.min_value:
            axes['ly'] = 0
        if abs(axes['rx']) < self.min_value:
            axes['rx'] = 0
        if abs(axes['ry']) < self.min_value:
            axes['ry'] = 0
        
        if self.chassis_type == 'Mecanum':
            axes['lx'] = misc.map(axes['lx'], -1, 1, -160, 160)
            axes['ly'] = misc.map(axes['ly'], 1, -1, -160, 160)
            axes['ry'] = misc.map(axes['ry'], -1, 1, -30, 30)
            v, d = self.Mecanum.translation(axes['ly'], axes['lx'], fake=True)
            msg = Mecanum()
            msg.velocity = v
            msg.direction = d
            msg.angular_rate = axes['ry']
            self.chassis_pub.publish(msg)

        elif self.chassis_type == 'Tank':
            axes['lx'] = misc.map(axes['lx'], -1, 1, -160, 160)
            axes['ry'] = misc.map(axes['ry'], -1, 1, -50, 50)
            v, d = self.Mecanum.translation(0.0, axes['lx'], fake=True)
            msg = Mecanum()
            msg.velocity = v
            msg.direction = d
            msg.angular_rate = axes['ry']
            self.chassis_pub.publish(msg)

        elif self.chassis_type == 'Slide_Rails':
            axes['ly'] = misc.map(axes['ly'], -1, 1, 350, -350)
            self.set_slideway(int(axes['ly']))       
        
        elif self.chassis_type == 'Conveyor_Belt':
            axes['ly'] = misc.map(axes['ly'], -1, 1, 100, -100)
            data = MotorState()
            data.id = 1
            data.rps = float(axes['ly'])
            msg = MotorsState()
            msg.data.append(data)
            self.motor_pub.publish(msg)



    def joy_callback(self):
        
        # 检查当前是否插入(check if currently inserted)
        self.count += 1
        if self.count > 10:
            if os.path.exists("/dev/input/js0"):
                if self.joy is None:
                    time.sleep(1)
                    pg.joystick.init()
                    if pg.joystick.get_count() > 0:
                        try:
                            self.joy = pg.joystick.Joystick(0)
                            self.joy.init()
                        except Exception as e:
                            self.get_logger().error(str(e)) 
            else:
                if self.joy is not None:
                    try:
                        self.joy.quit()
                        pg.joystick.quit()
                        self.joy = None
                    except Exception as e:
                        self.get_logger().error(str(e))

        if self.joy is None:
            return

        pg.event.pump()
        buttons = list(self.joy.get_button(i) for i in range(self.joy.get_numbuttons()))
        hat = list(self.joy.get_hat(0))
        axes = list(-self.joy.get_axis(i) for i in range(4))

        # 摇杆值(joystick value)
        axes = dict(zip(AXES_MAP, axes))
        # 摇杆 ad 值转为 bool 值(convert the ad value of joystick to bool value)
        laxis_l, laxis_r = 1 if axes['ly'] > 0.5 else 0, 1 if axes['ly'] < -0.5 else 0
        laxis_u, laxis_d = 1 if axes['lx'] > 0.5 else 0, 1 if axes['lx'] < -0.5 else 0
        raxis_l, raxis_r = 1 if axes['ry'] > 0.5 else 0, 1 if axes['ry'] < -0.5 else 0
        raxis_u, raxis_d = 1 if axes['rx'] > 0.5 else 0, 1 if axes['rx'] < -0.5 else 0

        # 方向帽的 ad 值转为 bool 值(convert the ad value of direction cap to bool value)
        hat_y, hat_x = hat
        hat_xl, hat_xr = 1 if hat_x > 0.5 else 0, 1 if hat_x < -0.5 else 0
        hat_yd, hat_yu = 1 if hat_y > 0.5 else 0, 1 if hat_y < -0.5 else 0

        buttons.extend([hat_xl, hat_xr, hat_yu, hat_yd])
        buttons.extend([laxis_l, laxis_r, laxis_u, laxis_d, raxis_l, raxis_r, raxis_u, raxis_d])
        buttons = dict(zip(BUTTON_MAP, buttons))
        # print(buttons)

        # 判断摇杆值的是否改变(determine whether the joystick value is changed)
        axes_changed = False
        for key, value in axes.items():  # 轴的值被改变(the axle value is changed)
            if abs(self.last_axes[key] - value) > 0.04:
                axes_changed = True
        if axes_changed:
            try:
                if self.chassis_type != 'None':
                    self.axes_callback(axes)
            except Exception as e:
                self.get_logger().error(str(e)) 

        for key, value in buttons.items():
            if value != self.last_buttons[key]:
                new_state = ButtonState.Pressed if value > 0 else ButtonState.Released
            else:
                new_state = ButtonState.Holding if value > 0 else ButtonState.Normal
            callback = "".join([key, '_callback'])
            if new_state != ButtonState.Normal:
                if hasattr(self, callback):

                    # self.get_logger().info('\033[1;32m%s\033[0m' % key)
                    try:
                        getattr(self, callback)(new_state)
                    except Exception as e:
                        self.get_logger().error(str(e)) 

        self.last_buttons = buttons
        self.last_axes = axes

def main():
    node = JoystickController('joystick_control')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
if __name__ == "__main__":
    main()


