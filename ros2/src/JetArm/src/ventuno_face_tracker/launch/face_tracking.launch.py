import os

from ament_index_python.packages import (
    get_package_share_directory
)
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource
)
from launch_ros.actions import Node


def generate_launch_description():
    robot_controller_path = get_package_share_directory(
        'ros_robot_controller'
    )

    peripherals_path = get_package_share_directory(
        'peripherals'
    )

    robot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                robot_controller_path,
                'launch',
                'ros_robot_controller.launch.py'
            )
        )
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                peripherals_path,
                'launch',
                'depth_camera.launch.py'
            )
        )
    )

    face_tracker = Node(
        package='ventuno_face_tracker',
        executable='face_tracker',
        name='ventuno_face_tracker',
        output='screen',
        parameters=[{
            'start_armed': False,

            'pan_servo_id': 1,
            'tilt_servo_id': 4,

            'pan_center': 500,
            'tilt_center': 500,

            'pan_min': 350,
            'pan_max': 650,
            'tilt_min': 350,
            'tilt_max': 650,

            'deadband_x': 45,
            'deadband_y': 35,

            'pan_gain': 0.025,
            'tilt_gain': 0.025,

            'max_step': 5,
            'command_interval': 0.12,
            'movement_duration': 0.18,

            'invert_pan': False,
            'invert_tilt': False,
        }]
    )

    return LaunchDescription([
        robot_controller,
        camera,
        face_tracker,
    ])
