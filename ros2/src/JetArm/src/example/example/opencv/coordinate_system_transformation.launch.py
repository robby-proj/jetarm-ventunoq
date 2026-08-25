import os
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    compiled = os.environ['need_compile']
    enable_display = LaunchConfiguration('enable_display', default='false')
    enable_display_arg = DeclareLaunchArgument('enable_display', default_value=enable_display)
    if compiled == 'True':
        example_package_path = get_package_share_directory('example')
    else:
        example_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/example'
    
    color_recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(example_package_path, 'example/opencv/color_recognition.launch.py')),
        launch_arguments={
            'enable_display': enable_display
        }.items()
    )
    coordinate_system_transformation_node = Node(
        package='example',
        executable='coordinate_system_transformation',
        output='screen',
    )

    return [color_recognition_launch,
            coordinate_system_transformation_node,
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
