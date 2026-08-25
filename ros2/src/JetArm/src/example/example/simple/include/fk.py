import rclpy
from rclpy.node import Node
import signal
from std_srvs.srv import Trigger
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from servo_controller_msgs.msg import ServoPosition, ServosPosition


class FkDemo(Node):
    def __init__(self):
        super().__init__('fk_demo')  # 初始化节点
        self.fk = ForwardKinematics(debug=True)

        # 创建总线舵机发布者
        self.servos_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        #等待机械臂底层控制服务启动
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(Trigger, '/kinematics/init_finish')
        self.client.wait_for_service()

        # 输出舵机脉宽值和正运动学解
        self.servo_list = [500, 400, 300, 400, 500]
        self.duration = 1.0
        self.get_logger().info("舵机脉宽值：{}".format(self.servo_list))

        pulse = transform.pulse2angle([i for i in self.servo_list])  # 舵机脉宽值转为弧度
        self.get_logger().info("舵机脉宽值转为弧度：{}".format(pulse)) 

        res = self.fk.get_fk(pulse)  # 获取运动学正解
        self.get_logger().info('正运动学解-坐标：{}'.format(res[0]))
        self.get_logger().info('正运动学解-四元数：{}'.format(res[1]))
        self.get_logger().info('转换四元数成rpy：{}'.format(transform.qua2rpy(res[1])))

        self.set_servo_position(self.servos_pub, self.duration, self.servo_list)#发布舵机信息，驱动机械臂
    
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
    fk_demo_node = FkDemo()  # 创建节点
    rclpy.spin(fk_demo_node)  # 保持节点运行
    fk_demo_node.destroy_node()  # 销毁节点
    rclpy.shutdown()  # 关闭 rclpy

if __name__ == '__main__':
    main()

