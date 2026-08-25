#!/usr/bin/env python3

import time
from typing import Optional

import cv2
import mediapipe as mp
import rclpy

from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool

from ros_robot_controller_msgs.msg import ServoPosition
from ros_robot_controller_msgs.msg import ServosPosition


class VentunoFaceTracker(Node):
    """MediaPipe face tracker with direct JetArm servo control and debug viewer."""

    def __init__(self) -> None:
        super().__init__('ventuno_face_tracker')

        # Topics
        self.declare_parameter(
            'image_topic',
            '/depth_cam/rgb/image_raw',
        )
        self.declare_parameter(
            'debug_image_topic',
            '/ventuno_face_tracker/debug_image',
        )
        self.declare_parameter(
            'servo_topic',
            '/ros_robot_controller/bus_servo/set_position',
        )

        # Safety
        self.declare_parameter('start_armed', False)

        # Servo mapping
        self.declare_parameter('pan_servo_id', 1)
        self.declare_parameter('tilt_servo_id', 4)

        self.declare_parameter('pan_center', 500)
        self.declare_parameter('tilt_center', 500)

        self.declare_parameter('pan_min', 350)
        self.declare_parameter('pan_max', 650)
        self.declare_parameter('tilt_min', 350)
        self.declare_parameter('tilt_max', 650)

        # Direction
        self.declare_parameter('invert_pan', True)
        self.declare_parameter('invert_tilt', True)

        # Tracking behavior
        self.declare_parameter('deadband_x', 40)
        self.declare_parameter('deadband_y', 35)
        self.declare_parameter('pan_gain', 0.030)
        self.declare_parameter('tilt_gain', 0.020)
        self.declare_parameter('max_step', 5)
        self.declare_parameter('command_interval', 0.15)
        self.declare_parameter('movement_duration', 0.20)

        # Detection
        self.declare_parameter('detection_confidence', 0.50)
        self.declare_parameter('detection_model', 0)
        self.declare_parameter('smoothing_alpha', 0.30)

        # Logging / viewer
        self.declare_parameter('no_face_log_interval', 3.0)
        self.declare_parameter('publish_debug_image', True)

        self.image_topic = str(
            self.get_parameter('image_topic').value
        )
        self.debug_image_topic = str(
            self.get_parameter('debug_image_topic').value
        )
        self.servo_topic = str(
            self.get_parameter('servo_topic').value
        )

        self.armed = bool(
            self.get_parameter('start_armed').value
        )

        self.pan_servo_id = int(
            self.get_parameter('pan_servo_id').value
        )
        self.tilt_servo_id = int(
            self.get_parameter('tilt_servo_id').value
        )

        self.pan_center = int(
            self.get_parameter('pan_center').value
        )
        self.tilt_center = int(
            self.get_parameter('tilt_center').value
        )

        self.pan_position = self.pan_center
        self.tilt_position = self.tilt_center

        self.bridge = CvBridge()

        self.filtered_face_x: Optional[float] = None
        self.filtered_face_y: Optional[float] = None

        self.last_command_time = 0.0
        self.last_no_face_log_time = 0.0

        detection_model = int(
            self.get_parameter('detection_model').value
        )
        detection_confidence = float(
            self.get_parameter('detection_confidence').value
        )

        self.face_detector = mp.solutions.face_detection.FaceDetection(
            model_selection=detection_model,
            min_detection_confidence=detection_confidence,
        )

        self.servo_publisher = self.create_publisher(
            ServosPosition,
            self.servo_topic,
            10,
        )

        self.debug_image_publisher = self.create_publisher(
            Image,
            self.debug_image_topic,
            10,
        )

        self.image_subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            qos_profile_sensor_data,
        )

        self.enable_service = self.create_service(
            SetBool,
            '~/enable',
            self.enable_callback,
        )

        self.center_service = self.create_service(
            SetBool,
            '~/center',
            self.center_callback,
        )

        self.get_logger().info(
            f'Listening for images on {self.image_topic}'
        )
        self.get_logger().info(
            f'Publishing servo commands on {self.servo_topic}'
        )
        self.get_logger().info(
            f'Publishing debug images on {self.debug_image_topic}'
        )

        if self.armed:
            self.get_logger().warning(
                'Face tracking started ARMED'
            )
        else:
            self.get_logger().warning(
                'Face tracking started DISARMED. Enable with: '
                'ros2 service call /ventuno_face_tracker/enable '
                'std_srvs/srv/SetBool "{data: true}"'
            )

    def enable_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        self.armed = bool(request.data)
        response.success = True

        if self.armed:
            response.message = 'Face tracking enabled'
            self.get_logger().warning('Face tracking ENABLED')
        else:
            response.message = 'Face tracking disabled'
            self.get_logger().warning('Face tracking DISABLED')

        return response

    def center_callback(
        self,
        request: SetBool.Request,
        response: SetBool.Response,
    ) -> SetBool.Response:
        if not request.data:
            response.success = False
            response.message = 'Send data: true to center servos'
            return response

        self.pan_position = self.pan_center
        self.tilt_position = self.tilt_center

        self.publish_servo_positions(
            self.pan_position,
            self.tilt_position,
            duration=1.0,
        )

        response.success = True
        response.message = (
            f'Centered pan={self.pan_position}, '
            f'tilt={self.tilt_position}'
        )
        return response

    @staticmethod
    def clamp(
        value: int,
        minimum: int,
        maximum: int,
    ) -> int:
        return max(minimum, min(maximum, value))

    def calculate_step(
        self,
        error: float,
        deadband: int,
        gain: float,
    ) -> int:
        if abs(error) <= deadband:
            return 0

        max_step = int(
            self.get_parameter('max_step').value
        )

        step = int(round(error * gain))

        if step == 0:
            step = 1 if error > 0 else -1

        return self.clamp(
            step,
            -max_step,
            max_step,
        )

    def publish_servo_positions(
        self,
        pan_position: int,
        tilt_position: int,
        duration: Optional[float] = None,
    ) -> None:
        message = ServosPosition()

        if duration is None:
            duration = float(
                self.get_parameter('movement_duration').value
            )

        message.duration = float(duration)

        pan = ServoPosition()
        pan.id = int(self.pan_servo_id)
        pan.position = int(pan_position)

        tilt = ServoPosition()
        tilt.id = int(self.tilt_servo_id)
        tilt.position = int(tilt_position)

        message.position = [pan, tilt]
        self.servo_publisher.publish(message)

    @staticmethod
    def select_largest_face(detections):
        if not detections:
            return None

        return max(
            detections,
            key=lambda detection: (
                detection.location_data
                .relative_bounding_box.width
                * detection.location_data
                .relative_bounding_box.height
            ),
        )

    def smooth_face_center(
        self,
        face_x: float,
        face_y: float,
    ):
        alpha = float(
            self.get_parameter('smoothing_alpha').value
        )

        if self.filtered_face_x is None:
            self.filtered_face_x = face_x
            self.filtered_face_y = face_y
        else:
            self.filtered_face_x = (
                alpha * face_x
                + (1.0 - alpha) * self.filtered_face_x
            )
            self.filtered_face_y = (
                alpha * face_y
                + (1.0 - alpha) * self.filtered_face_y
            )

        return self.filtered_face_x, self.filtered_face_y

    def publish_debug_image(
        self,
        frame,
        source_message: Image,
    ) -> None:
        enabled = bool(
            self.get_parameter('publish_debug_image').value
        )
        if not enabled:
            return

        try:
            debug_message = self.bridge.cv2_to_imgmsg(
                frame,
                encoding='bgr8',
            )
            debug_message.header = source_message.header
            self.debug_image_publisher.publish(debug_message)
        except Exception as exc:
            self.get_logger().error(
                f'Could not publish debug image: {exc}'
            )

    def draw_common_overlay(
        self,
        frame,
        width: int,
        height: int,
    ) -> None:
        center_x = width // 2
        center_y = height // 2

        deadband_x = int(
            self.get_parameter('deadband_x').value
        )
        deadband_y = int(
            self.get_parameter('deadband_y').value
        )

        cv2.line(
            frame,
            (center_x, 0),
            (center_x, height),
            (255, 255, 255),
            1,
        )
        cv2.line(
            frame,
            (0, center_y),
            (width, center_y),
            (255, 255, 255),
            1,
        )

        cv2.rectangle(
            frame,
            (
                center_x - deadband_x,
                center_y - deadband_y,
            ),
            (
                center_x + deadband_x,
                center_y + deadband_y,
            ),
            (0, 255, 255),
            2,
        )

        state = 'ARMED' if self.armed else 'DISARMED'
        state_color = (
            (0, 255, 0)
            if self.armed
            else (0, 0, 255)
        )

        cv2.putText(
            frame,
            state,
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.9,
            state_color,
            2,
        )

        cv2.putText(
            frame,
            f'pan={self.pan_position} tilt={self.tilt_position}',
            (20, 70),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

    def image_callback(self, message: Image) -> None:
        try:
            rgb_frame = self.bridge.imgmsg_to_cv2(
                message,
                desired_encoding='rgb8',
            )
        except Exception as exc:
            self.get_logger().error(
                f'Image conversion failed: {exc}'
            )
            return

        if rgb_frame is None or rgb_frame.size == 0:
            self.get_logger().warning(
                'Received empty camera frame'
            )
            return

        height, width = rgb_frame.shape[:2]

        debug_frame = cv2.cvtColor(
            rgb_frame,
            cv2.COLOR_RGB2BGR,
        )

        self.draw_common_overlay(
            debug_frame,
            width,
            height,
        )

        results = self.face_detector.process(rgb_frame)
        detection = self.select_largest_face(
            results.detections
        )

        if detection is None:
            cv2.putText(
                debug_frame,
                'NO FACE',
                (20, 110),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.9,
                (0, 0, 255),
                2,
            )

            self.publish_debug_image(
                debug_frame,
                message,
            )

            now = time.monotonic()
            interval = float(
                self.get_parameter(
                    'no_face_log_interval'
                ).value
            )

            if now - self.last_no_face_log_time >= interval:
                self.get_logger().info(
                    'No face detected'
                )
                self.last_no_face_log_time = now

            return

        box = detection.location_data.relative_bounding_box

        x1 = max(0, int(box.xmin * width))
        y1 = max(0, int(box.ymin * height))
        x2 = min(
            width - 1,
            int((box.xmin + box.width) * width),
        )
        y2 = min(
            height - 1,
            int((box.ymin + box.height) * height),
        )

        if x2 <= x1 or y2 <= y1:
            self.publish_debug_image(
                debug_frame,
                message,
            )
            return

        raw_face_x = (x1 + x2) / 2.0
        raw_face_y = (y1 + y2) / 2.0

        face_x, face_y = self.smooth_face_center(
            raw_face_x,
            raw_face_y,
        )

        frame_center_x = width / 2.0
        frame_center_y = height / 2.0

        error_x = face_x - frame_center_x
        error_y = face_y - frame_center_y

        cv2.rectangle(
            debug_frame,
            (x1, y1),
            (x2, y2),
            (0, 255, 0),
            2,
        )

        cv2.circle(
            debug_frame,
            (int(face_x), int(face_y)),
            7,
            (0, 0, 255),
            -1,
        )

        cv2.line(
            debug_frame,
            (
                int(frame_center_x),
                int(frame_center_y),
            ),
            (
                int(face_x),
                int(face_y),
            ),
            (255, 0, 255),
            2,
        )

        cv2.putText(
            debug_frame,
            f'error x={error_x:.0f} y={error_y:.0f}',
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        cv2.putText(
            debug_frame,
            f'face x={face_x:.0f} y={face_y:.0f}',
            (20, 145),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 0),
            2,
        )

        self.publish_debug_image(
            debug_frame,
            message,
        )

        if not self.armed:
            return

        now = time.monotonic()
        command_interval = float(
            self.get_parameter('command_interval').value
        )

        if now - self.last_command_time < command_interval:
            return

        deadband_x = int(
            self.get_parameter('deadband_x').value
        )
        deadband_y = int(
            self.get_parameter('deadband_y').value
        )
        pan_gain = float(
            self.get_parameter('pan_gain').value
        )
        tilt_gain = float(
            self.get_parameter('tilt_gain').value
        )

        pan_step = self.calculate_step(
            error_x,
            deadband_x,
            pan_gain,
        )
        tilt_step = self.calculate_step(
            error_y,
            deadband_y,
            tilt_gain,
        )

        if bool(
            self.get_parameter('invert_pan').value
        ):
            pan_step *= -1

        if bool(
            self.get_parameter('invert_tilt').value
        ):
            tilt_step *= -1

        if pan_step == 0 and tilt_step == 0:
            return

        pan_min = int(
            self.get_parameter('pan_min').value
        )
        pan_max = int(
            self.get_parameter('pan_max').value
        )
        tilt_min = int(
            self.get_parameter('tilt_min').value
        )
        tilt_max = int(
            self.get_parameter('tilt_max').value
        )

        self.pan_position = self.clamp(
            self.pan_position + pan_step,
            pan_min,
            pan_max,
        )
        self.tilt_position = self.clamp(
            self.tilt_position + tilt_step,
            tilt_min,
            tilt_max,
        )

        self.publish_servo_positions(
            self.pan_position,
            self.tilt_position,
        )

        self.last_command_time = now

        self.get_logger().info(
            f'Face error x={error_x:.0f}, '
            f'y={error_y:.0f} | '
            f'pan={self.pan_position}, '
            f'tilt={self.tilt_position}'
        )

    def destroy_node(self) -> None:
        self.face_detector.close()
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = VentunoFaceTracker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
