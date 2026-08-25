import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    compiled = os.environ['need_compile']
    camera_type = os.environ['CAMERA_TYPE']
    depth_camera_name_arg = DeclareLaunchArgument(
        'depth_camera_name',
        default_value='depth_cam'
    )


    app_arg = DeclareLaunchArgument(
        'app',
        default_value='false'
    )

    if compiled == 'True':
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        peripherals_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/peripherals'

    if camera_type == 'GEMINI':
        camera_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/include/gemini.launch.py')),
            launch_arguments={
                'camera_name': LaunchConfiguration('depth_camera_name'),
            }.items())
    else:
        camera_launch = GroupAction([
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(peripherals_package_path, 'launch/include/usb_cam.launch.py'))
                ),

            Node(
                package='tf2_ros',
                executable='static_transform_publisher',
                name="depth_cam_base_link",
                arguments=[
                    '--x', '0',
                    '--y', '0',
                    '--z', '0',
                    '--qx', '0',
                    '--qy', '0',
                    '--qz', '0',
                    '--qw', '1',
                    '--frame-id', 'depth_cam_link',
                    '--child-frame-id', 'depth_cam_color_frame'
                ]
            ),

        ])

    return LaunchDescription([
        depth_camera_name_arg,
        app_arg,
        camera_launch
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
