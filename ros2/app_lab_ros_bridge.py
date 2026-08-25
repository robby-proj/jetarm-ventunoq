#!/usr/bin/env python3

import json
import socket
import threading
import time
from typing import Optional

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

from ros_robot_controller_msgs.msg import ServoPosition
from ros_robot_controller_msgs.msg import ServosPosition


class AppLabRosBridge(Node):
    """
    App Lab -> UDP -> ROS 2 JetArm bridge.

    This version intentionally uses the normal/conservative proportional
    face-tracking behavior. It keeps all App Lab/WebUI/voice-compatible UDP
    commands while removing predictive, distance-adaptive and 2D-vector
    movement logic.
    """

    def __init__(self) -> None:
        super().__init__("app_lab_face_tracking_bridge")

        # Network
        self.declare_parameter("listen_host", "0.0.0.0")
        self.declare_parameter("listen_port", 5600)

        # ROS / servos
        self.declare_parameter(
            "servo_topic",
            "/ros_robot_controller/bus_servo/set_position",
        )
        self.declare_parameter("pan_servo_id", 1)
        self.declare_parameter("tilt_servo_id", 4)

        # Conservative calibrated defaults
        self.declare_parameter("pan_center", 500)
        self.declare_parameter("tilt_center", 200)
        self.declare_parameter("pan_min", 425)
        self.declare_parameter("pan_max", 575)
        self.declare_parameter("tilt_min", 150)
        self.declare_parameter("tilt_max", 300)

        self.declare_parameter("invert_pan", True)
        self.declare_parameter("invert_tilt", True)

        # Normal tracking response
        self.declare_parameter("pan_gain", 55.0)
        self.declare_parameter("tilt_gain", 12.0)
        self.declare_parameter("deadband_x", 0.06)
        self.declare_parameter("deadband_y", 0.08)

        # Conservative smoothing
        self.declare_parameter("smoothing_alpha_x", 0.22)
        self.declare_parameter("smoothing_alpha_y", 0.15)

        # Per-update limits
        self.declare_parameter("pan_max_step", 10)
        self.declare_parameter("tilt_max_step", 5)
        self.declare_parameter("minimum_servo_change", 2)

        # Timing
        self.declare_parameter("control_interval", 0.03)
        self.declare_parameter("movement_duration", 0.06)
        self.declare_parameter("face_timeout", 0.5)
        self.declare_parameter("start_armed", False)

        # Runtime state
        self.armed = bool(self.get_parameter("start_armed").value)

        self.pan_position = int(self.get_parameter("pan_center").value)
        self.tilt_position = int(self.get_parameter("tilt_center").value)

        self.last_published_pan = self.pan_position
        self.last_published_tilt = self.tilt_position

        self.latest_detection: Optional[dict] = None
        self.last_detection_time = 0.0

        # Generation counter prevents the ROS timer from applying one UDP
        # detection multiple times before a fresh detector sample arrives.
        self.detection_generation = 0
        self.last_processed_generation = 0

        self.last_command_time = 0.0

        self.filtered_x: Optional[float] = None
        self.filtered_y: Optional[float] = None

        self.lock = threading.Lock()
        self.running = True

        self.publisher = self.create_publisher(
            ServosPosition,
            str(self.get_parameter("servo_topic").value),
            10,
        )

        self.enable_service = self.create_service(
            SetBool,
            "~/enable",
            self.enable_callback,
        )
        self.center_service = self.create_service(
            SetBool,
            "~/center",
            self.center_callback,
        )

        self.control_timer = self.create_timer(0.01, self.control_loop)

        self.socket_thread = threading.Thread(
            target=self.udp_loop,
            daemon=True,
        )
        self.socket_thread.start()

        self.get_logger().info("App Lab ROS bridge started DISARMED")
        self.get_logger().info(
            "NORMAL conservative face tracking ENABLED "
            "(no prediction / no distance boost / no vector motion)"
        )
        self.get_logger().info(
            "Enable from WebUI or voice command through App Lab"
        )

    @staticmethod
    def clamp(value, minimum, maximum):
        return max(minimum, min(maximum, value))

    def clear_detection(self) -> None:
        with self.lock:
            self.latest_detection = None
            self.last_detection_time = 0.0
            self.last_processed_generation = self.detection_generation

        self.filtered_x = None
        self.filtered_y = None

    def enable_callback(self, request, response):
        self.armed = bool(request.data)

        if not self.armed:
            self.clear_detection()

        response.success = True
        response.message = (
            "Tracking enabled" if self.armed else "Tracking disabled"
        )

        self.get_logger().warning(response.message.upper())
        return response

    def center_callback(self, request, response):
        if not request.data:
            response.success = False
            response.message = "Send data: true to center"
            return response

        self.center_robot()
        response.success = True
        response.message = (
            f"Centered pan={self.pan_position}, tilt={self.tilt_position}"
        )
        return response

    def center_robot(self) -> None:
        self.armed = False
        self.clear_detection()

        self.pan_position = int(self.get_parameter("pan_center").value)
        self.tilt_position = int(self.get_parameter("tilt_center").value)

        self.publish_positions(duration=1.0)

        self.last_published_pan = self.pan_position
        self.last_published_tilt = self.tilt_position

        self.get_logger().warning("CENTER COMMAND RECEIVED")

    def udp_loop(self) -> None:
        host = str(self.get_parameter("listen_host").value)
        port = int(self.get_parameter("listen_port").value)

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind((host, port))
        sock.settimeout(0.5)

        self.get_logger().info(
            f"Listening for App Lab messages on UDP {host}:{port}"
        )

        while self.running and rclpy.ok():
            try:
                data, sender = sock.recvfrom(8192)
                message = json.loads(data.decode("utf-8"))

                message_type = message.get(
                    "message_type",
                    message.get("type", "detection"),
                )

                if message_type == "control":
                    command = str(message.get("command", ""))

                    if command == "set_tracking":
                        enabled = bool(
                            message.get(
                                "enabled",
                                message.get("tracking_enabled", False),
                            )
                        )

                        self.armed = enabled

                        if not enabled:
                            self.clear_detection()

                        self.get_logger().warning(
                            "WEBUI/VOICE SET TRACKING: "
                            f"{'ENABLED' if enabled else 'DISABLED'} "
                            f"from {sender[0]}:{sender[1]}"
                        )

                    elif command == "center":
                        self.center_robot()

                    elif command == "emergency_stop":
                        self.armed = False
                        self.clear_detection()
                        self.get_logger().error(
                            "EMERGENCY STOP RECEIVED FROM APP LAB"
                        )

                    elif command == "request_status":
                        self.get_logger().info(
                            "APP LAB REQUESTED ROBOT STATUS: "
                            f"armed={self.armed}, "
                            f"pan={self.pan_position}, "
                            f"tilt={self.tilt_position}"
                        )

                    else:
                        self.get_logger().warning(
                            f"UNKNOWN APP LAB COMMAND: {command!r}"
                        )

                    continue

                if message_type != "detection":
                    continue

                if (
                    "normalized_x" not in message
                    or "normalized_y" not in message
                ):
                    continue

                normalized_x = float(message["normalized_x"])
                normalized_y = float(message["normalized_y"])

                if not (
                    0.0 <= normalized_x <= 1.0
                    and 0.0 <= normalized_y <= 1.0
                ):
                    continue

                with self.lock:
                    self.latest_detection = message
                    self.last_detection_time = time.monotonic()
                    self.detection_generation += 1

            except socket.timeout:
                continue
            except Exception as exc:
                self.get_logger().warning(f"UDP receive error: {exc}")

        sock.close()

    def smooth_coordinates(
        self,
        normalized_x: float,
        normalized_y: float,
    ) -> tuple[float, float]:
        alpha_x = float(
            self.get_parameter("smoothing_alpha_x").value
        )
        alpha_y = float(
            self.get_parameter("smoothing_alpha_y").value
        )

        alpha_x = self.clamp(alpha_x, 0.0, 1.0)
        alpha_y = self.clamp(alpha_y, 0.0, 1.0)

        if self.filtered_x is None or self.filtered_y is None:
            self.filtered_x = normalized_x
            self.filtered_y = normalized_y
        else:
            self.filtered_x = (
                alpha_x * normalized_x
                + (1.0 - alpha_x) * self.filtered_x
            )
            self.filtered_y = (
                alpha_y * normalized_y
                + (1.0 - alpha_y) * self.filtered_y
            )

        return self.filtered_x, self.filtered_y

    def calculate_step(
        self,
        error: float,
        gain: float,
        deadband: float,
        max_step: int,
    ) -> int:
        if abs(error) <= deadband:
            return 0

        step = int(round(error * gain))

        if step == 0:
            step = 1 if error > 0 else -1

        return int(self.clamp(step, -max_step, max_step))

    def control_loop(self) -> None:
        if not self.armed:
            return

        now = time.monotonic()

        with self.lock:
            detection = (
                dict(self.latest_detection)
                if self.latest_detection is not None
                else None
            )
            detection_time = self.last_detection_time
            generation = self.detection_generation

        if detection is None:
            return

        # Critical stability rule:
        # one servo correction per fresh UDP detection.
        if generation == self.last_processed_generation:
            return

        age = now - detection_time

        if age > float(self.get_parameter("face_timeout").value):
            return

        if (
            now - self.last_command_time
            < float(self.get_parameter("control_interval").value)
        ):
            return

        try:
            normalized_x = float(detection["normalized_x"])
            normalized_y = float(detection["normalized_y"])
        except (KeyError, TypeError, ValueError):
            self.last_processed_generation = generation
            return

        filtered_x, filtered_y = self.smooth_coordinates(
            normalized_x,
            normalized_y,
        )

        error_x = filtered_x - 0.5
        error_y = filtered_y - 0.5

        pan_step = self.calculate_step(
            error_x,
            float(self.get_parameter("pan_gain").value),
            float(self.get_parameter("deadband_x").value),
            int(self.get_parameter("pan_max_step").value),
        )

        tilt_step = self.calculate_step(
            error_y,
            float(self.get_parameter("tilt_gain").value),
            float(self.get_parameter("deadband_y").value),
            int(self.get_parameter("tilt_max_step").value),
        )

        if bool(self.get_parameter("invert_pan").value):
            pan_step *= -1

        if bool(self.get_parameter("invert_tilt").value):
            tilt_step *= -1

        # Mark this detector sample consumed even if it falls in the deadband.
        self.last_processed_generation = generation

        if pan_step == 0 and tilt_step == 0:
            self.last_command_time = now
            return

        proposed_pan = int(
            self.clamp(
                self.pan_position + pan_step,
                int(self.get_parameter("pan_min").value),
                int(self.get_parameter("pan_max").value),
            )
        )

        proposed_tilt = int(
            self.clamp(
                self.tilt_position + tilt_step,
                int(self.get_parameter("tilt_min").value),
                int(self.get_parameter("tilt_max").value),
            )
        )

        minimum_change = int(
            self.get_parameter("minimum_servo_change").value
        )

        if (
            abs(proposed_pan - self.last_published_pan) < minimum_change
            and
            abs(proposed_tilt - self.last_published_tilt) < minimum_change
        ):
            self.pan_position = proposed_pan
            self.tilt_position = proposed_tilt
            self.last_command_time = now
            return

        self.pan_position = proposed_pan
        self.tilt_position = proposed_tilt

        self.publish_positions()

        self.last_published_pan = self.pan_position
        self.last_published_tilt = self.tilt_position
        self.last_command_time = now

        self.get_logger().info(
            f"raw=({normalized_x:.3f},{normalized_y:.3f}) "
            f"filtered=({filtered_x:.3f},{filtered_y:.3f}) "
            f"step=({pan_step},{tilt_step}) "
            f"pan={self.pan_position} tilt={self.tilt_position}"
        )

    def publish_positions(
        self,
        duration: Optional[float] = None,
    ) -> None:
        message = ServosPosition()

        message.duration = float(
            duration
            if duration is not None
            else self.get_parameter("movement_duration").value
        )

        pan_servo = ServoPosition()
        pan_servo.id = int(
            self.get_parameter("pan_servo_id").value
        )
        pan_servo.position = int(self.pan_position)

        tilt_servo = ServoPosition()
        tilt_servo.id = int(
            self.get_parameter("tilt_servo_id").value
        )
        tilt_servo.position = int(self.tilt_position)

        message.position = [pan_servo, tilt_servo]
        self.publisher.publish(message)

    def destroy_node(self) -> None:
        self.running = False

        if (
            hasattr(self, "socket_thread")
            and self.socket_thread.is_alive()
        ):
            self.socket_thread.join(timeout=1.0)

        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AppLabRosBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
