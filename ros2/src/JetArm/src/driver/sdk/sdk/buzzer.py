import rclpy
import time
from rclpy.node import Node
from ros_robot_controller_msgs.msg import BuzzerState

class BuzzerController(Node):
    
    def __init__(self, name='buzzer_controller'):
        super().__init__(name)
        self.pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)

    def set_buzzer(self, freq, on_time, off_time, repeat):
        msg = BuzzerState()
        msg.freq = freq
        msg.on_time = on_time
        msg.off_time = off_time
        msg.repeat = repeat
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    buzzer_controller = BuzzerController()

    # 示例调用
    while rclpy.ok():
        buzzer_controller.set_buzzer(500, 0.1, 0.5, 1)
        time.sleep(1.0)
        buzzer_controller.set_buzzer(1500, 0.1, 0.5, 2)
        time.sleep(1.0)

    buzzer_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

