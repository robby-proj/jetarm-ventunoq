#!/usr/bin/env python3
# coding: utf8

import os
import cv2
import numpy as np
import sdk.pid as pid
from sdk import common
from sdk.common import colors, point_remapped, box_center, distance, get_area_max_contour, range_rgb


class ColorTracker:
    def __init__(self, target_color):
        self.tracker_type = 'color'
        self.target_color = target_color
        self.pid_yaw = pid.PID(50.5, 0.0, 0.000003)
        self.pid_pitch = pid.PID(0.045, 0, 0.00015)
        self.yaw = 500
        self.pitch = 0.28
    
        self.data = common.get_yaml_data("/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app/config/lab_config.yaml")  
        self.lab_data = self.data['/**']['ros__parameters'] 
    def proc (self, source_image, result_image):
        h, w = source_image.shape[:2]

        img_blur = cv2.GaussianBlur(source_image, (3, 3), 3) # 高斯模糊(Gaussian blur)
        img_lab = cv2.cvtColor(img_blur, cv2.COLOR_RGB2LAB) # 转换到 LAB 空间(convert to the LAB space)
        mask = cv2.inRange(img_lab, tuple(self.lab_data['color_range_list'][self.target_color]['min']), tuple(self.lab_data['color_range_list'][self.target_color]['max']))  # 二值化

        # 平滑边缘，去除小块，合并靠近的块(smooth edges, remove small patches, and merge nearby patches)
        eroded = cv2.erode(mask, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))
        dilated = cv2.dilate(eroded, cv2.getStructuringElement(cv2.MORPH_RECT, (3, 3)))

        # 找出最大轮廓(find out the largest contour)
        contours = cv2.findContours(dilated, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)[-2]
        max_contour_area = get_area_max_contour(contours, 100) # 获取最大面积对应轮廓

        # 如果有符合要求的轮廓(if there are contours that meet the requirements)
        if max_contour_area[0] is not None :
            (center_x, center_y), radius = cv2.minEnclosingCircle(max_contour_area[0]) # 最小外接圆(the minimum bounding rectangle)

            # 圈出识别的的要追踪的色块(encircle the recognized color blocks to be tracked)
            # circle_color = colors.rgb[self.target_color] if self.target_color in colors.rgb else (0x55, 0x55, 0x55)
            circle_color = range_rgb[self.target_color]
            cv2.circle(result_image, (int(center_x), int(center_y)), int(radius), circle_color, 2)

            center_x = center_x / w
            if abs(center_x - 0.5) > 0.03: # 相差范围小于一定值就不用再动了(if the difference is less than a certain value, there's no need to move further)
                self.pid_yaw.SetPoint = 0.5 # 我们的目标是要让色块在画面的中心, 就是整个画面的像素宽度的 1/2 位置(our goal is to have the color block at the center of the frame, which is at half of the pixel width of the entire frame)
                self.pid_yaw.update(center_x)
                self.yaw = min(max(self.yaw + self.pid_yaw.output, 200), 800)
            else:
                self.pid_yaw.clear() # 如果已经到达中心了就复位一下 pid 控制器(if it's already at the center, reset the PID controller)

            center_y = center_y / h
            if abs(center_y - 0.5) > 0.015:
                self.pid_pitch.SetPoint = 0.5
                self.pid_pitch.update(center_y)
                self.pitch = self.pitch + self.pid_pitch.output
                # 限制 self.pitch 的值在 0.2 到 0.3 之间
                self.pitch = max(0.23, min(0.30, self.pitch))
            else:
                self.pid_pitch.clear()
            return (result_image, (self.pitch, self.yaw))
        else:
            return (result_image, None)


