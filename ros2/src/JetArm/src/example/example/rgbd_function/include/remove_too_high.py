#!/usr/bin/python3
#coding=utf8

# 等高保持(maintain equal height)
# 找出识别区域高度最高的物体(find the tallest object in the recognition area)
# 如果物体高度超过阈值(if the object height exceeds the threshold)
# 则将最高的物体移除(remove the tallest object)
# 程序测试使用的是 30x30x30mm的木块和 40x40x40mm的木块(the program is tested using a 30x30x30mm wooden block and a 40x40x40mm wooden block)

import cv2
import math
import time
import rclpy
import queue
import signal
import threading
import numpy as np
import message_filters
from rclpy.node import Node
from sdk import  common, fps
from std_srvs.srv import SetBool
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image, CameraInfo
from rclpy.executors import MultiThreadedExecutor
from servo_controller_msgs.msg import ServosPosition
from ros_robot_controller_msgs.msg import BuzzerState
from rclpy.callback_groups import ReentrantCallbackGroup
from kinematics.kinematics_control import set_pose_target
from kinematics_msgs.srv import GetRobotPose, SetRobotPose
from servo_controller.bus_servo_control import set_servo_position


def depth_pixel_to_camera(pixel_coords, depth, intrinsics):
    fx, fy, cx, cy = intrinsics
    px, py = pixel_coords
    x = (px - cx) * depth / fx
    y = (py - cy) * depth / fy
    z = depth
    return np.array([x, y, z])


class RemoveTooHighObjectNode(Node):

    hand2cam_tf_matrix = [
    [0.0, 0.0, 1.0, -0.101],
    [-1.0, 0.0, 0.0, 0.01],
    [0.0, -1.0, 0.0, 0.05],
    [0.0, 0.0, 0.0, 1.0]
]
    def __init__(self, name):
        rclpy.init()
        super().__init__(name, allow_undeclared_parameters=True, automatically_declare_parameters_from_overrides=True)
        self.endpoint = None
        self.fps = fps.FPS()
        self.last_shape = "none"
        self.moving = False
        self.running = True
        self.stamp = time.time()
        self.count = 0
        self.last_position = (0, 0, 0)

        self.image_queue = queue.Queue(maxsize=2)
        self.joints_pub = self.create_publisher(ServosPosition, '/servo_controller', 1) # 舵机控制
        
        timer_cb_group = ReentrantCallbackGroup()
        self.create_service(Trigger, '~/start', self.start_srv_callback) # 进入玩法
        self.create_service(Trigger, '~/stop', self.stop_srv_callback, callback_group=timer_cb_group) # 退出玩法
        self.client = self.create_client(Trigger, '/controller_manager/init_finish')
        self.client.wait_for_service()
        self.client = self.create_client(SetBool, '/depth_cam/set_ldp_enable')
        self.client.wait_for_service()


        self.buzzer_pub = self.create_publisher(BuzzerState, '/ros_robot_controller/set_buzzer', 1)
        self.get_current_pose_client = self.create_client(GetRobotPose, '/kinematics/get_current_pose')
        self.get_current_pose_client.wait_for_service()
        self.set_pose_target_client = self.create_client(SetRobotPose, '/kinematics/set_pose_target')
        self.set_pose_target_client.wait_for_service()

        
        
        rgb_sub = message_filters.Subscriber(self, Image, '/depth_cam/rgb/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/depth_cam/depth/image_raw')
        info_sub = message_filters.Subscriber(self, CameraInfo, '/depth_cam/depth/camera_info')

        # 同步时间戳, 时间允许有误差在0.02s
        sync = message_filters.ApproximateTimeSynchronizer([rgb_sub, depth_sub, info_sub], 3, 0.08)
        sync.registerCallback(self.multi_callback) #执行反馈函数
        
        timer_cb_group = ReentrantCallbackGroup()
        self.timer = self.create_timer(0.0, self.init_process, callback_group=timer_cb_group)


    def init_process(self):
        self.timer.cancel()

        set_servo_position(self.joints_pub, 1, ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 200)))
        time.sleep(2)

        threading.Thread(target=self.main, daemon=True).start()
        self.create_service(Trigger, '~/init_finish', self.get_node_state)
        self.get_logger().info('\033[1;32m%s\033[0m' % 'start')

    def get_node_state(self, request, response):
        response.success = True
        return response

    def shutdown(self, signum, frame):
        self.running = False


    def start_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "start")
        self.start = True
        response.success = True
        response.message = "start"
        return response

    def stop_srv_callback(self, request, response):
        self.get_logger().info('\033[1;32m%s\033[0m' % "stop")
        self.start = False
        self.moving = False
        self.count = 0
        self.last_pitch_yaw = (0, 0)
        self.last_position = (0, 0, 0)
        set_servo_position(self.joints_pub, 1, ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 200)))
        response.success = True
        response.message = "stop"
        return response

    def send_request(self, client, msg):
        future = client.call_async(msg)
        while rclpy.ok():
            if future.done() and future.result():
                return future.result()

    def multi_callback(self, ros_rgb_image, ros_depth_image, depth_camera_info):
        if self.image_queue.full():
            # 如果队列已满，丢弃最旧的图像
            self.image_queue.get()
        # 将图像放入队列
        self.image_queue.put((ros_rgb_image, ros_depth_image, depth_camera_info))

    def get_endpoint(self):
        endpoint = self.send_request(self.get_current_pose_client, GetRobotPose.Request()).pose
        self.endpoint = common.xyz_quat_to_mat([endpoint.position.x, endpoint.position.y, endpoint.position.z],
                                        [endpoint.orientation.w, endpoint.orientation.x, endpoint.orientation.y, endpoint.orientation.z])
        return self.endpoint

    def pick(self, position, angle):
        angle =  angle % 90
        angle = angle - 90 if angle > 40 else (angle + 90 if angle < -45 else angle)
        yaw = 80
        msg = BuzzerState()
        msg.freq = 1900
        msg.on_time = 0.2
        msg.off_time = 0.01
        msg.repeat = 1
        self.buzzer_pub.publish(msg)
        time.sleep(1)
        position[2] += 0.05
        msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((1, servo_data[0]),(2, servo_data[1]), (3, servo_data[2]),(4, servo_data[3])))
            time.sleep(1.5)
        angle = 500 + int(1000 * (angle + res.rpy[-1]) / 240)
        set_servo_position(self.joints_pub, 0.5, ((5, angle),))
        time.sleep(0.5)

        position[2] -= 0.05        
        msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((1, servo_data[0]),(2, servo_data[1]), (3, servo_data[2]),(4, servo_data[3])))
            time.sleep(1.5)
        set_servo_position(self.joints_pub, 1.0, ((10, 600),))
        time.sleep(1)
        position[2] += 0.05
        msg = set_pose_target(position, yaw, [-180.0, 180.0], 1.0)
        res = self.send_request(self.set_pose_target_client, msg)
        if res.pulse:
            servo_data = res.pulse
            set_servo_position(self.joints_pub, 1.5, ((1, servo_data[0]),(2, servo_data[1]), (3, servo_data[2]),(4, servo_data[3])))
            time.sleep(1.5)
        set_servo_position(self.joints_pub, 1.0, ((1, 500), (2, 740), (3, 100), (4, 260), (5, 500)))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1.0, ((1, 150), (2, 635), (3, 100), (4, 260), (5, 500)))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1.0,  ((1, 150), (2, 600), (3, 125), (4, 175), (5, 500)))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1.0,  ((1, 150), (2, 600), (3, 125), (4, 175), (5, 500), (10, 200)))
        time.sleep(1)
        set_servo_position(self.joints_pub, 1.0,  ((1, 500), (2, 540), (3, 220), (4, 50), (5, 500), (10, 200)))

        time.sleep(1)
        self.stamp = time.time()
        self.moving = False


    
    def main(self):
        while self.running:
            try:
                ros_rgb_image, ros_depth_image, depth_camera_info = self.image_queue.get(block=True, timeout=1)
            except queue.Empty:
                if not self.running:
                    break
                else:
                    continue
            try:
                rgb_image = np.ndarray(shape=(ros_rgb_image.height, ros_rgb_image.width, 3), dtype=np.uint8, buffer=ros_rgb_image.data)
                depth_image = np.ndarray(shape=(ros_depth_image.height, ros_depth_image.width), dtype=np.uint16, buffer=ros_depth_image.data)
                bgr_image = cv2.cvtColor(rgb_image[40:440, ], cv2.COLOR_RGB2BGR)
                K = depth_camera_info.k


                ih, iw = depth_image.shape[:2]

            
                depth = depth_image.copy()
                depth[380:400, :] = np.array([[55555,]*640]*20)
                depth = depth.reshape((-1, )).copy()
                depth[depth<=100] = 55555
                min_index = np.argmin(depth)
                min_y = min_index // iw
                min_x = min_index - min_y * iw

                min_dist = depth_image[min_y, min_x]
                sim_depth_image = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
                depth_image = np.where(depth_image > min_dist + 10, 0, depth_image)
                sim_depth_image_sort = np.clip(depth_image, 0, 2000).astype(np.float64) / 2000 * 255
                depth_gray = sim_depth_image_sort.astype(np.uint8)
                depth_gray = cv2.GaussianBlur(depth_gray, (5, 5), 0)
                _, depth_bit = cv2.threshold(depth_gray, 1, 255, cv2.THRESH_BINARY)
                depth_bit = cv2.erode(depth_bit, np.ones((5, 5), np.uint8))
                depth_bit = cv2.dilate(depth_bit, np.ones((3, 3), np.uint8))
                depth_color_map = cv2.applyColorMap(sim_depth_image.astype(np.uint8), cv2.COLORMAP_JET)

                contours, hierarchy = cv2.findContours(depth_bit, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
                shape = 'none'
                txt = ""
                z = 0
                largest = None
                for obj in contours:
                    area = cv2.contourArea(obj)
                    if area < 500 or self.moving is True:
                        continue
                    (cx, cy), radius = cv2.minEnclosingCircle(obj)
                    cv2.circle(depth_color_map, (int(cx), int(cy)), int(radius), (0, 0, 255), 2)
                    
                    self.get_endpoint()
                    position = depth_pixel_to_camera((cx, cy), depth_image[int(cy), int(cx)] / 1000.0, (K[0], K[4], K[2], K[5]))
                    position[2] = position[2] + 0.03

                    pose_end = np.matmul(self.hand2cam_tf_matrix, common.xyz_euler_to_mat(position, (0, 0, 0)))  # 转换的末端相对坐标(relative coordinates of the converted end)
                    world_pose = np.matmul(self.endpoint, pose_end)  # 转换到机械臂世界坐标(convert to the robotic arm's world coordinates)
                    pose_t, pose_R = common.mat_to_xyz_euler(world_pose)
                    pose_t[1] += 0.01
                    if pose_t[2] > z:
                        largest = obj, pose_t
                        min_x = cx
                        min_y = cy
                        txt = 'Dist: {}mm'.format(depth_image[int(cy), int(cx)])
                        cv2.circle(depth_color_map, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                        cv2.circle(depth_color_map, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                        cv2.putText(depth_color_map, txt, (11, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                        cv2.putText(depth_color_map, txt, (10, ih-20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                        cv2.circle(bgr_image, (int(min_x), int(min_y)), 8, (32, 32, 32), -1)
                        cv2.circle(bgr_image, (int(min_x), int(min_y)), 6, (255, 255, 255), -1)
                        cv2.putText(bgr_image, txt, (11, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (32, 32, 32), 6, cv2.LINE_AA)
                        cv2.putText(bgr_image, txt, (10, ih - 20), cv2.FONT_HERSHEY_PLAIN, 2.0, (240, 240, 240), 2, cv2.LINE_AA)

                if largest is not None:
                    obj, pose_t = largest
                    dist = math.sqrt((self.last_position[0] - pose_t[0]) ** 2 + (self.last_position[1] - pose_t[1])** 2 + (self.last_position[2] - pose_t[2])**2)
                    self.last_position = pose_t
                    self.get_logger().info("dist: " + str(pose_t[2]))
                    if dist < 0.002 and 0.01 < pose_t[2] :
                        if time.time() - self.stamp > 0.5:
                            self.stamp = time.time()
                            rect = cv2.minAreaRect(obj)
               
                            # time.sleep(2)
                            self.moving = True
                            threading.Thread(target=self.pick, args=(pose_t, rect[2])).start()

                    else:
                        self.stamp = time.time()
                

                self.fps.update()
                # bgr_image = self.fps.show_fps(bgr_image)
                result_image = np.concatenate([bgr_image, depth_color_map], axis=1)
                cv2.imshow("depth", result_image)
                key = cv2.waitKey(1)
                if key != -1:
                    rospy.signal_shutdown('shutdown1')

            except Exception as e:
                self.get_logger().info('error1: ' + str(e))
        rclpy.shutdown()

def main():
    node = RemoveTooHighObjectNode('remove_too_high')
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()

if __name__ == "__main__":
    main()

