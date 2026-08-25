#!/usr/bin/env python3
# encoding: utf-8
# @data:2023/03/20
# @author:aiden
# 实时获取角度反馈，根据当前位置和
# 目标位置的最小差值来获取最优解
import time
import math
import rclpy
import numpy as np
from sdk import common
from rclpy.node import Node
from std_srvs.srv import Trigger
from geometry_msgs.msg import Pose
import kinematics.transform as transform
from kinematics.forward_kinematics import ForwardKinematics
from kinematics.inverse_kinematics import get_ik, set_link, get_link, set_joint_range, get_joint_range
from servo_controller_msgs.msg import ServoStateList, ServosPosition
from kinematics_msgs.msg import JointsRange, Link
from kinematics_msgs.srv import SetRobotPose, SetJointValue, GetRobotPose, SetLink, GetLink, SetJointRange, GetJointRange
from servo_controller.bus_servo_control import set_servo_position

fk = ForwardKinematics(debug=False)  # 不开启打印
class SearchKinematicsSolutionsNode(Node):
    def __init__(self, name):
        # 初始化节点
        rclpy.init()
        super().__init__(name)
        self.name = name
        self.last_time = 0
        self.current_time = 0
        self.current_servo_positions = []

        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1)

        self.create_subscription(ServoStateList, '/controller_manager/servo_states', self.get_servo_position, 1)

        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()

        self.create_service(SetLink, '~/set_link', self.set_link_srv) 
        self.create_service(GetLink, '~/get_link', self.get_link_srv) 
        self.create_service(SetJointRange, '~/set_joint_range', self.set_joint_range_srv) 
        self.create_service(GetJointRange, '~/get_joint_range', self.get_joint_range_srv) 
        self.create_service(SetRobotPose, '~/set_pose_target', self.set_pose_target_srv)
        self.create_service(SetRobotPose, '~/set_pose_target_smooth', self.set_pose_target_smooth_srv)
        self.create_service(GetRobotPose, '~/get_current_pose', self.get_current_pose_srv) 
        self.create_service(SetJointValue, '~/set_joint_value_target', self.set_joint_value_target_srv) 

        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def set_link_srv(self, request, response):
        # 设置link长度
        base_link = request.data.base_link
        link1 = request.data.link1
        link2 = request.data.link2
        link3 = request.data.link3
        end_effector_link = request.data.end_effector_link
        set_link(base_link, link1, link2, link3, end_effector_link)
        fk.set_link(base_link, link1, link2, link3, end_effector_link)
        response.success = True
        response.message = "set_link"
        return response

    def get_link_srv(self, request, response):
        # 获取各个link长度
        data = get_link()
        data1 = fk.get_link()
        link = Link()
        
        response.success = True
        if data == data1:
            link.base_link = data[0]
            link.link1 = data[1]
            link.link2 = data[2]
            link.link3 = data[3]
            link.end_effector_link = data[4]
            response.data = link
        else:
            response.data = []

        return response

    def set_joint_range_srv(self, request, response):
        # 设置关节范围
        joint1 = request.data.joint1
        joint2 = request.data.joint2
        joint3 = request.data.joint3
        joint4 = request.data.joint4
        joint5 = request.data.joint5
        set_joint_range([joint1.min, joint1.max], [joint2.min, joint2.max], [joint3.min, joint3.max], [joint4.min, joint4.max], [joint5.min, joint5.max], 'deg')
        fk.set_joint_range([joint1.min, joint1.max], [joint2.min, joint2.max], [joint3.min, joint3.max], [joint4.min, joint4.max], [joint5.min, joint5.max], 'deg')
        response.success = True
        response.message = "set_joint_range"
        return response

    def get_joint_range_srv(self, request, response):
        # 获取各个关节范围
        data = get_joint_range('deg')
        data1 = fk.get_joint_range('deg')
        joint_range = JointsRange()
        joint_range.joint1.min = data[0][0]
        joint_range.joint1.max = data[0][1]
        joint_range.joint2.min = data[1][0]
        joint_range.joint2.max = data[1][1]
        joint_range.joint3.min = data[2][0]
        joint_range.joint3.max = data[2][1]
        joint_range.joint4.min = data[3][0]
        joint_range.joint4.max = data[3][1]
        joint_range.joint5.min = data[4][0]
        joint_range.joint5.max = data[4][1]
        response.success = True
        if data == data1:
            response.data = joint_range
        else:
            response.data = []
        return response

    def set_joint_value_target_srv(self, request, response):
        # 正运动学解
        joint_value = request.joint_value
        angle = transform.pulse2angle(joint_value)
        res = fk.get_fk(angle)
        pose = Pose() 
        response.success = True
        if res:
            pose.position.x = res[0][0]
            pose.position.y = res[0][1]
            pose.position.z = res[0][2]
            pose.orientation = res[1]
            response.solution = True
        else:
            response.solution = False
        response.pose = pose
        return response

    def get_current_pose_srv(self, request, response):
        # 获取机械臂当前位置
        angle = transform.pulse2angle(self.current_servo_positions)
        res = fk.get_fk(angle)
        pose = Pose() 
        response.success = True
        if res:
            pose.position.x = res[0][0]
            pose.position.y = res[0][1]
            pose.position.z = res[0][2]
            pose.orientation = res[1]
            response.solution = True
        else:
            response.solution = False
        response.pose = pose
        return response

    def get_servo_position(self, msg):
        # 获取舵机当前角度
        servo_states = []
        for i in msg.servo_state:
            if 0 < i.id < 6:
                servo_states.append(i.position)
        self.current_servo_positions = np.array(servo_states)
        # self.get_logger().info(str(self.current_servo_positions))

    def set_pose_target(self, position, pitch, pitch_range, resolution):
        # 逆运动学解，获取最优解(所有电机转动最小)
        position = list(position)

        all_solutions = get_ik(position, pitch, list(pitch_range), resolution)
        if len(all_solutions) > 0 and len(self.current_servo_positions) > 0:
            rpy = []
            min_d = 1000.0*5
            optimal_solution = []
            for s in all_solutions:
                pulse_solutions = transform.angle2pulse(s[0], True)
                try:
                    for i in pulse_solutions:
                        d = np.array(i) - self.current_servo_positions
                        d_abs = np.maximum(d, -d)
                        min_sum = np.sum(d_abs)
                        if min_sum < min_d:
                            min_d = float(min_sum)
                            for k in range(len(i)):
                                if i[k] < 0:
                                    i[k] = 0
                                elif i[k] > 1000:
                                    i[k] = 1000
                            rpy = s[1]
                            optimal_solution = i
                except BaseException as e:
                    self.get_logger().info('\033[1;32m%s\033[0m' % 'choose solution error')
            # self.get_logger().info('\033[1;32m%s\033[0m' % str(time.time() - t1))
            return [True, optimal_solution, self.current_servo_positions.tolist(), rpy, min_d]
        else:
            return [True, [], [], [], 0.0]

    def set_pose_target_srv(self, request, response):
        # self.get_logger().info('\033[1;32mset_pose_target: %s\033[0m' % str(request))
        position, pitch, pitch_range, resolution = request.position, request.pitch, request.pitch_range, request.resolution
        
        # self.last_time = self.current_time
        res = self.set_pose_target(position, pitch, pitch_range, resolution)
        
        response.success = True
        response.pulse = res[1]
        response.current_pulse = res[2]
        response.rpy = res[3]
        response.min_variation = res[4]

        # self.current_time = time.time()
        # self.ttt = self.current_time - self.last_time
        # self.get_logger().info('\033[1;32m%s\033[0m' %"ttt: " + str(self.ttt))
        # self.get_logger().info('\033[1;32mset_pose_target: %s\033[0m' % str(response))
        return response

    def quintic_polynomial_interpolation(self, start_pos, end_pos, T, dt=0.02):
        """
        五次多项式插值，生成机械臂末端位置轨迹。
        参数:
        - start_pos: 起始位置 (3,) 数组
        - end_pos: 结束位置 (3,) 数组
        - start_vel: 起始速度 (3,) 数组
        - end_vel: 结束速度 (3,) 数组
        - start_acc: 起始加速度 (3,) 数组
        - end_acc: 结束加速度 (3,) 数组
        - T: 轨迹持续时间
        - dt: 时间步长
        返回:
        - t: 时间数组
        - trajectory: 每个时间步对应的末端位置轨迹 (num_points, 3)
        """
        start_vel = np.array([0.0, 0.0, 0.0])  # 起始速度
        end_vel = np.array([0.0, 0.0, 0.0])    # 结束速度
        start_acc = np.array([0, 0, 0])  # 起始加速度
        end_acc = np.array([0.0, 0.0, 0.0])    # 结束加速度

        num_points = int(T / dt)
        t = np.linspace(0, T, num_points)  # 时间数组
        trajectory = np.zeros((num_points, len(start_pos)))  # 存储轨迹

        for i in range(len(start_pos)):
            # 五次多项式系数计算
            a0 = start_pos[i]
            a1 = start_vel[i]
            a2 = start_acc[i] / 2
            a3 = (20 * (end_pos[i] - start_pos[i]) - 
                  (8 * end_vel[i] + 12 * start_vel[i]) * T - 
                  (3 * start_acc[i] - end_acc[i]) * T**2) / (2 * T**3)
            a4 = (-30 * (end_pos[i] - start_pos[i]) + 
                  (14 * end_vel[i] + 16 * start_vel[i]) * T + 
                  (3 * start_acc[i] - 2 * end_acc[i]) * T**2) / (2 * T**4)
            a5 = (12 * (end_pos[i] - start_pos[i]) - 
                  (6 * end_vel[i] + 6 * start_vel[i]) * T - 
                  (start_acc[i] - end_acc[i]) * T**2) / (2 * T**5)

            # 计算该维度的轨迹
            trajectory[:, i] = (a0 + a1 * t + a2 * t**2 + a3 * t**3 + 
                                a4 * t**4 + a5 * t**5)

        return trajectory

    def quintic_polynomial_interpolation_fixed_distance(self, start_pos, end_pos, step, dt=0.02):
        start_vel = np.array([0, 0, 0])  # 起始速度
        end_vel = np.array([0, 0, 0])    # 结束速度
        start_acc = np.array([0, 0, 0])  # 起始加速度
        end_acc = np.array([0, 0, 0])    # 结束加速度

        # 计算直线距离，估算轨迹长度
        total_distance = np.linalg.norm(np.array(end_pos) - np.array(start_pos))
        
        # 计算需要的步数
        num_points = int(np.ceil(total_distance / step))
        
        T = dt*num_points
        t = np.linspace(0, T, num_points + 1)  
        trajectory = np.zeros((num_points + 1, len(start_pos)))  # 存储轨迹

        # 对每个维度计算五次多项式
        for i in range(len(start_pos)):
            # 五次多项式系数计算
            a0 = start_pos[i]
            a1 = start_vel[i]
            a2 = start_acc[i] / 2
            a3 = (20 * (end_pos[i] - start_pos[i]) - 
                  (8 * end_vel[i] + 12 * start_vel[i]) * T - 
                  (3 * start_acc[i] - end_acc[i]) * T**2) / (2 * T**3)
            a4 = (-30 * (end_pos[i] - start_pos[i]) + 
                  (14 * end_vel[i] + 16 * start_vel[i]) * T + 
                  (3 * start_acc[i] - 2 * end_acc[i]) * T**2) / (2 * T**4)
            a5 = (12 * (end_pos[i] - start_pos[i]) - 
                  (6 * end_vel[i] + 6 * start_vel[i]) * T - 
                  (start_acc[i] - end_acc[i]) * T**2) / (2 * T**5)

            # 计算该维度的轨迹
            trajectory[:, i] = (a0 + a1 * t + a2 * t**2 + a3 * t**3 + 
                                a4 * t**4 + a5 * t**5)

        return trajectory

    def set_pose_target_smooth_srv(self, request, response):
        position, pitch, pitch_range, resolution = request.position, request.pitch, request.pitch_range, request.resolution
        # self.get_logger().info(f'{self.current_servo_positions}')
        angle = transform.pulse2angle(self.current_servo_positions)
        res = fk.get_fk(angle)
        if res:
            start_pose = res[0]
            current_pitch = math.degrees(common.qua2rpy(res[1])[1])
            # self.get_logger().info('\033[1;32mset_pose_target: %s\033[0m' % str(current_pitch))
            # self.get_logger().info('\033[1;32mstart_pose: %s\033[0m' % str(start_pose))
            # self.get_logger().info('\033[1;32mposition: %s\033[0m' % str(position))
            # result = self.quintic_polynomial_interpolation(start_pose, position, request.duration)
            result = self.quintic_polynomial_interpolation(start_pose, position, request.duration)
            angle = []
            t = time.time()
            # self.get_logger().info('\033[1;32mresult: %s\033[0m' % str(result))
            dt = (current_pitch - pitch)/len(result)
            for i in range(len(result)):
                p = current_pitch - i*dt
                res = self.set_pose_target(result[i], p, pitch_range, resolution)
                if res[1]:
                    angle.append(res[1])
                current_pulse = res[2]
                rpy = res[3]
                min_variation = res[4]
            # self.get_logger().info('time: %s' % str(time.time() - t))
            # self.get_logger().info('\033[1;32mangle: %s\033[0m' % str(angle))
            # for i in angle:
                # set_servo_position(self.joints_pub, 0, ((1, i[0]), (2, i[1]), (3, i[2]), (4, i[3])))
                # time.sleep(0.02)
            angle = np.array(angle)
            angle = angle.flatten().tolist()

            response.success = True
            response.pulse = angle
            response.current_pulse = res[2]
            response.rpy = res[3]
            response.min_variation = res[4]
        else:
            response.success = False
        return response

def main():
    node = SearchKinematicsSolutionsNode('kinematics')
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
