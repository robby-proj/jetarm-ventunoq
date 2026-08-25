#!/usr/bin/python3
# coding=utf8
import os
import cv2
import time
import sdk.pid as pid

class ObjectTracker:
    def __init__(self, use_mouse=False):
        self.use_mouse = use_mouse
        if self.use_mouse:
            name = 'track'
            cv2.namedWindow(name, 1)
            cv2.setMouseCallback(name, self.onmouse)
        self.params = cv2.TrackerNano_Params()
        model_path = os.path.split(os.path.realpath(__file__))[0]
        self.params.backbone = os.path.join(model_path, 'resources/models/nanotrack_backbone_sim.onnx')
        self.params.neckhead = os.path.join(model_path, 'resources/models/nanotrack_head_sim.onnx')
        self.tracker = cv2.TrackerNano_create(self.params)
        self.mouse_click = False
        self.selection = None  # 实时跟踪鼠标的跟踪区域
        self.track_window = None  # 要检测的物体所在区域
        self.drag_start = None  # 标记，是否开始拖动鼠标
        self.start_circle = True
        self.start_click = False

        self.joint4_dis = 233

        self.y_dis = 500
        self.z_dis = 0.15
        
        self.joint_pid = pid.PID(0.0, 0.0, 0.0)
        self.z_pid = pid.PID(0.0, 0.0, 0.0)#pid初始化(pid initialization)
        self.y_pid = pid.PID(0.0, 0.0, 0.0)

    def set_init_param(self, joint4_dis, y_dis, z_dis): 
        self.joint4_dis = joint4_dis
        self.y_dis = y_dis
        self.z_dis = z_dis

    def update_pid(self, p1, p2, p3):
        self.joint_pid = pid.PID(p1[0], p1[1], p1[2])
        self.z_pid = pid.PID(p2[0], p2[1], p2[2])#pid初始化(pid initialization)
        self.y_pid = pid.PID(p3[0], p3[1], p3[2])

    # 鼠标点击事件回调函数
    def onmouse(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:  # 鼠标左键按下
            self.mouse_click = True
            self.drag_start = (x, y)  # 鼠标起始位置
            self.track_window = None
        if self.drag_start:  # 是否开始拖动鼠标，记录鼠标位置
            xmin = min(x, self.drag_start[0])
            ymin = min(y, self.drag_start[1])
            xmax = max(x, self.drag_start[0])
            ymax = max(y, self.drag_start[1])
            self.selection = (xmin, ymin, xmax, ymax)
        if event == cv2.EVENT_LBUTTONUP:  # 鼠标左键松开
            self.mouse_click = False
            self.drag_start = None
            self.track_window = self.selection
            self.selection = None
        if event == cv2.EVENT_RBUTTONDOWN:
            self.mouse_click = False
            self.selection = None  # 实时跟踪鼠标的跟踪区域
            self.track_window = None  # 要检测的物体所在区域
            self.drag_start = None  # 标记，是否开始拖动鼠标
            self.start_circle = True
            self.start_click = False
            self.tracker = cv2.TrackerNano_create(self.params)

    def set_track_target(self, target, image):
        self.tracker.init(image, target)

    def get_target(self, image):
        if self.start_circle and self.use_mouse:
            # 用鼠标拖拽一个框来指定区域
            if self.track_window:  # 跟踪目标的窗口画出后，实时标出跟踪目标
                cv2.rectangle(image, (self.track_window[0], self.track_window[1]),
                              (self.track_window[2], self.track_window[3]), (0, 0, 255), 2)
            elif self.selection:  # 跟踪目标的窗口随鼠标拖动实时显示
                cv2.rectangle(image, (self.selection[0], self.selection[1]), (self.selection[2], self.selection[3]),
                              (0, 255, 255), 2)
            if self.mouse_click:
                self.start_click = True
            if self.start_click:
                if not self.mouse_click:
                    self.start_circle = False
            if not self.start_circle:
                print('start tracking')
                bbox = (self.track_window[0], self.track_window[1], self.track_window[2] - self.track_window[0],
                        self.track_window[3] - self.track_window[1])
                self.tracker.init(image, bbox)
        else:
            ok, box = self.tracker.update(image)
            if ok and min(box) > 0:
                return image, box
            else:
                # Tracking failure
                cv2.putText(image, "Tracking failure detected !", (10, 460), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                            (255, 255, 0), 1)
        return image, None

    def track(self, image):
        image, box = self.get_target(image)
        if box is not None:
            img_h, img_w = image.shape[:2]
            p1 = (int(box[0]), int(box[1]))
            p2 = (int(p1[0] + box[2]), int(p1[1] + box[3]))

            cv2.rectangle(image, p1, p2, (0, 255, 0), 2, 1)
            center_x = (p1[0] + p2[0]) / 2
            center_y = (p1[1] + p2[1]) / 2

            x, y = center_x, center_y
            
            if self.z_dis <= 0.23:
                self.joint_pid.SetPoint = img_h / 2.0
                if abs(y - img_h / 2.0) < 25:
                    y = img_h / 2.0
                self.joint_pid.update(y)
                self.joint4_dis += self.joint_pid.output
                self.joint4_dis = 100 if self.joint4_dis < 100 else self.joint4_dis
                self.joint4_dis = 391 if self.joint4_dis > 391 else self.joint4_dis
            
            if self.joint4_dis >= 391 or self.z_dis > 0.23:
                self.z_pid.SetPoint = img_h / 2  # 设定(set)
                self.z_pid.update(y)  # 当前(current)
                self.z_dis += self.z_pid.output  # 输出(output)

                self.z_dis = 0.23 if self.z_dis < 0.23 else self.z_dis
                self.z_dis = 0.3 if self.z_dis > 0.3 else self.z_dis
            if abs(x - img_w/2.0) < 15:
                x = img_w / 2.0
            self.y_pid.SetPoint = img_w / 2.0
            self.y_pid.update(x)
            self.y_dis += self.y_pid.output

            self.y_dis = 200 if self.y_dis < 200 else self.y_dis
            self.y_dis = 800 if self.y_dis > 800 else self.y_dis
            # print(self.z_dis, y, self.joint4_dis)
        return int(self.y_dis), self.z_dis, int(self.joint4_dis), image

if __name__ == '__main__':
    cap = cv2.VideoCapture(-1)
    track = ObjectTracker(True)
    while True:
        try:
            ret, image = cap.read()
            if ret:
                x, y, frame = track.track(image)
                cv2.imshow('track', frame)
                cv2.waitKey(1)
            else:
                time.sleep(0.01)
        except KeyboardInterrupt:
            break
    cap.release()
    cv2.destroyAllWindows()


