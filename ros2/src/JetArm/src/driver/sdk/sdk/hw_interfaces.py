#!/usr/bin/env python3
#
# 提供机械臂控制板的 ROS 控制接口(provide the ROS control port of robotic arm control board)
#

import time
import threading
from ros_robot_controller.ros_robot_controller_sdk import Board


class HwInterfaces:
    def __init__(self, serial_port='/dev/ttyUSB0', baudrate=1000000, use_sim=False):
        self.use_sim = use_sim
        self.lock = threading.RLock()
        if not self.use_sim:
            self.board = Board()
            self.board.enable_reception()
        else:
            self.board = None

        self.buzzer_off()
        self.led_off()


    def set_led(self, on_ticks, off_ticks, repeat=0, led_id=2):
        if self.board is not None:
            with self.lock:
                self.board.set_led(on_ticks / 1000.0, off_ticks / 1000.0, repeat, led_id)

    def led_on(self, led_id=2):
        self.set_led(100, 0, 0, led_id)

    def led_off(self, led_id=2):
        self.set_led(0, 100, 0, led_id)

    def set_buzzer(self, freq, on_ticks, off_ticks, repeat):
        if self.board is not None:
            with self.lock:
                self.board.set_buzzer(freq, on_ticks / 1000.0, off_ticks / 1000.0, repeat)

    def buzzer_on(self, freq):
        self.set_buzzer(freq, 100, 0, 0)

    def buzzer_off(self):
        self.set_buzzer(0, 0, 100, 0)

    def serial_servo_id_set(self, servo_id, new_id):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_set_id(servo_id, new_id)

    def serial_servo_id_read(self, servo_id, retry=10):
        while retry > 0:
            try:
                ret = self.board.bus_servo_read_id(servo_id)
                if ret is not None:
                    return ret[0]
            except Exception as e:
                print(e)
            retry -= 1
            time.sleep(0.05)
        return None

    def serial_servo_move(self, servo_id, pos, dur):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_set_position(dur / 1000.0, ((servo_id, pos),))

    def serial_servo_set_servos(self, dur, positions):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_set_position(dur / 1000.0, positions)

    def serial_servo_stop(self, servo_id):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_stop(servo_id)

    def serial_servo_load(self, servo_id, enable):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_enable_torque(servo_id, enable)

    def serial_servo_dev_adj(self, servo_id, dev):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_set_offset(servo_id, dev)

    def serial_servo_dev_save(self, servo_id):
        if self.board is not None:
            with self.lock:
                self.board.bus_servo_save_offset(servo_id)

    def serial_servo_dev_read(self, servo_id, retry=10):
        while retry > 0:
            try:
                ret = self.board.bus_servo_read_offset(servo_id)
                if ret is not None:
                    return ret[0]
            except Exception as e:
                print(e)
            retry -= 1
            time.sleep(0.05)
        return None

    def serial_servo_pos_read(self, servo_id, retry=10):
        while retry > 0:
            try:
                ret = self.board.bus_servo_read_position(servo_id)
                if ret is not None:
                    return ret[0]
            except Exception as e:
                print(e)
            retry -= 1
            time.sleep(0.05)
        return None



