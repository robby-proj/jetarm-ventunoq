import rclpy
import time
from servo_controller import bus_servo_control 

def goto_left(pub, duration=1.5):
    bus_servo_control.set_servo_position(pub, int(duration * 1000), ((1, 875), (2, 610), (3, 70), (4, 140), (5, 500), (10, 200)))
    time.sleep(duration)

def goto_right(pub, duration=1.5):
    bus_servo_control.set_servo_position(pub, int(duration * 1000), ((1, 125), (2, 610), (3, 70), (4, 140), (5, 500), (10, 200)))
    time.sleep(duration)
   

def go_home(pub, duration=0.8):
    bus_servo_control.set_servo_position(pub, int(duration), ( (2, 560), (3, 130), (4, 115), (5, 500), (10, 200)))
    time.sleep(duration+0.2)
    bus_servo_control.set_servo_position(pub, int(duration), ((1, 500),))
    time.sleep(duration)
def goto_home(pub, duration=0.8):
    bus_servo_control.set_servo_position(pub, int(duration), ((1, 500),))
    time.sleep(duration+0.2)
    bus_servo_control.set_servo_position(pub, int(duration), ( (2, 560), (3, 130), (4, 115), (5, 500), (10, 200)))
    time.sleep(duration)
def goto_default(pub, duration=1.5):
    go_home(pub, duration)

def place(pub, duration=1.5):
    bus_servo_control.set_servo_position(pub, int(duration), ((1, 500), (2, 610), (3, 70), (4, 140), (5, 500), (10, 200)))

def go_back(pub, duration=0.8):
  
    bus_servo_control.set_servo_position(pub, int(duration), ((1, 500), (2, 560), (3, 130), (4, 115), (5, 500), (10, 550)))
    time.sleep(duration)
