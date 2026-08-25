import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.actions import ExecuteProcess, GroupAction, TimerAction
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']
    if compiled == 'True':
        sdk_package_path = get_package_share_directory('sdk')
        app_package_path = get_package_share_directory('app')
        peripherals_package_path = get_package_share_directory('peripherals')
    else:
        sdk_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/driver/sdk'
        app_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/app'
        peripherals_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/peripherals'


    sdk_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sdk_package_path, 'launch/jetarm_sdk.launch.py')),
    )

    
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(peripherals_package_path, 'launch/depth_camera.launch.py')),
    )


    rosbridge_websocket_launch = ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 'rosbridge_websocket_launch.xml'],
            output='screen'
        )

    web_video_server_node = Node(
        package='web_video_server',
        executable='web_video_server',
        output='screen',
    )
    startup_check_node = Node(
        package='app',
        executable='startup_check',
        output='screen',
    )


    start_app_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(app_package_path, 'launch/start_app.launch.py')),
    )


    joystick_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(peripherals_package_path, 'launch/joystick_control.launch.py')),
    )

    startup_check_node = Node(
        package='bringup',
        executable='startup_check',
        output='screen',
    )
    bringup_launch = GroupAction(
     actions=[
         start_app_launch,
         TimerAction(
             period=18.0,  # 延时等待其它节点启动好(delay and wait for other nodes to start up properly)
             actions=[depth_camera_launch],
         ),
      ]
    )
    return [
            sdk_launch,
            bringup_launch,
            startup_check_node,
            web_video_server_node,
            joystick_control_launch,
            rosbridge_websocket_launch,
            ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function = launch_setup)
    ])

if __name__ == '__main__':
    # 创建一个LaunchDescription对象
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
