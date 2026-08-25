#!/usr/bin/env python3

import time

import rclpy
from rclpy.node import Node

from ros_robot_controller_msgs.msg import ServoPosition
from ros_robot_controller_msgs.msg import ServosPosition


class FixedPickNode(Node):
    """
    Simple fixed-position pick-and-place demo for JetArm.

    Confirmed mapping:
      ID 1  = base rotation
      ID 10 = gripper

    IDs 2-5 must be tuned for the physical joints on this specific arm.
    """

    def __init__(self):
        super().__init__("fixed_pick_node")

        self.publisher = self.create_publisher(
            ServosPosition,
            "/ros_robot_controller/bus_servo/set_position",
            10,
        )

        # Allow DDS discovery before publishing.
        time.sleep(1.0)

    def move(self, positions: dict[int, int], duration: float = 1.5):
        """
        Move one or more servos.

        Example:
            move({1: 500, 10: 850}, duration=2.0)
        """
        msg = ServosPosition()
        msg.duration = float(duration)

        for servo_id, position in positions.items():
            servo = ServoPosition()
            servo.id = int(servo_id)
            servo.position = int(position)
            msg.position.append(servo)

        self.publisher.publish(msg)

        self.get_logger().info(
            f"Moving servos: {positions}, duration={duration:.1f}s"
        )

        # Give the arm enough time to finish the movement.
        time.sleep(duration + 0.4)

    def open_gripper(self):
        # Tune between approximately 700 and 900.
        self.move({10: 700}, duration=1.0)

    def close_gripper(self):
        # Start conservatively. Do not force the gripper.
        self.move({10: 300}, duration=1.0)

    def home(self):
        """
        Conservative example home position.

        Tune IDs 2-5 individually before using the complete sequence.
        """
        self.move(
            {
                1: 500,
                2: 500,
                3: 500,
                4: 500,
                5: 500,
                10: 850,
            },
            duration=2.5,
        )


    def run_pick_sequence(self):
        self.get_logger().info("Starting verified pick sequence")

    # Open gripper before approaching.
        self.move({10: 350}, duration=1.0)

    # Rotate base toward object.
        self.move({1: 120}, duration=1.5)

    # Extend arm toward object.
        self.move({3: 100}, duration=1.5)

    # Set wrist/gripper angle.
        self.move({4: 200}, duration=1.5)

    # Lower around object.
        self.move({3: 45}, duration=1.5)

    # Close gripper.
        self.move({10: 600}, duration=1.0)

        self.get_logger().info("Object grasp attempted")

    # Pause so you can verify that the object is secure.
        time.sleep(2.0)

    # Lift by reversing the last arm movement.
        self.move({3: 200}, duration=1.5)

    # Raise wrist slightly for clearance.
        self.move({4: 500}, duration=1.5)

        self.get_logger().info("Object lifted")


def main():
    rclpy.init()

    node = FixedPickNode()

    try:
        node.run_pick_sequence()
    except KeyboardInterrupt:
        node.get_logger().warning("Sequence interrupted")
    except Exception as exc:
        node.get_logger().error(f"Sequence failed: {exc}")
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
