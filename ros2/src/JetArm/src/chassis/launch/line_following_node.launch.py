from ament_index_python.packages import get_package_share_directory
import os

from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, OpaqueFunction, GroupAction, DeclareLaunchArgument

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        chassis_package_path = get_package_share_directory('chassis')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        sdk_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/driver/sdk'
        chassis_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/chassis'
        peripherals_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/peripherals'
    line_following_node = GroupAction([

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
            ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')),
            ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(chassis_package_path, 'launch/chassis_controller_node.launch.py')),
            ),

        Node(
            package='chassis',
            executable='line_following',
            output='screen',
            ),
    ])

    return [
            line_following_node,
            ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象(create a LaunchDescription object)
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
