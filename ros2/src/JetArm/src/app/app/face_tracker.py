#!/usr/bin/env python3
# coding: utf8

import sys
import cv2
import sdk.pid as pid
import mediapipe as mp
from sdk.common import show_faces, mp_face_location, box_center, distance
from math import radians


class FaceTracker:
    def __init__(self):
        self.tracker_type = "face"
        self.face_detector = mp.solutions.face_detection.FaceDetection(
            min_detection_confidence=0.5,
        )

        self.pid_yaw = pid.PID(55.5, 0, 1.2)
        self.pid_pitch = pid.PID(45.5, 0, 1.2)
        self.detected_face = 0 
        self.yaw = 500
        self.pitch = 350

    def proc(self, source_image, result_image):
        results = self.face_detector.process(source_image)
        boxes, keypoints = mp_face_location(results, source_image)
        o_h, o_w = source_image.shape[:2]

        if len(boxes) > 0:
            self.detected_face += 1 
            self.detected_face = min(self.detected_face, 20) # 让计数总是不大于20(ensure that the count never exceeds 20)

            # 连续 5 帧识别到了人脸就开始追踪, 避免误识别(start tracking if a face is detected in five consecutive frames to avoid false recognition)
            if self.detected_face >= 5:
                center = [box_center(box) for box in boxes] # 计算所有人脸的中心坐标(calculate the center coordinates of all human faces)
                dist = [distance(c, (o_w / 2, o_h / 2)) for c in center] # 计算所有人脸中心坐标到画面中心的距离(calculate the distance from the center of each detected face to the center of the image)
                face = min(zip(boxes, center, dist), key=lambda k: k[2]) # 找出到画面中心距离最小的人脸(identify the face with the minimum distance to the center of the image)

                # 计算要追踪的人脸距画面中心的x轴距离(0~1)。(calculate the x-axis distance (0~1) of the face to be tracked from the center of the image)
                c_x, c_y = face[1]
                dist_x = c_x / o_w
                dist_y = c_y / o_h

                if abs(dist_y - 0.5) > 0.01:
                    self.pid_pitch.SetPoint = 0.5
                    self.pid_pitch.update(dist_y) # 更新俯仰角 pid 控制器(update the pitch angle PID controller)
                    self.pitch = min(max(self.pitch + self.pid_pitch.output, 100), 740)  # 获取新的俯仰角并限制运动范围(obtain the new pitch angle and limit the range of motion)
                else:
                    self.pid_pitch.clear()

                if abs(dist_x - 0.5) > 0.01:
                    self.pid_yaw.SetPoint = 0.5
                    self.pid_yaw.update(dist_x) # 更新偏航角 pid 控制器(update the yaw angle PID controller)
                    self.yaw = min(max(self.yaw + self.pid_yaw.output, 0),  1000)  # 获取新的偏航角并限制运动范围(get the new yaw angle and limit the range of motion)
                else:
                    self.pid_yaw.clear()

        else: # 这里是没有识别到人脸的处理(here is the processing for when no face is detected)
            # gc.collect()
            if self.detected_face > 0:
                self.detected_face -= 1
            else:
                self.pid_pitch.clear()
                self.pid_yaw.clear()

        result_image = show_faces(source_image, result_image, boxes, keypoints) # 在画面中显示识别到的人脸和脸部关键点(display the detected faces and facial key points on the image)
        return result_image, (self.pitch, self.yaw)

