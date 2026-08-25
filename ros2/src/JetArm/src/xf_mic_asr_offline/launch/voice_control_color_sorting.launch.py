import os
from ament_index_python.packages import get_package_share_directory

from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription, LaunchService
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def launch_setup(context):
    compiled = os.environ['need_compile']
    start = LaunchConfiguration('start', default='true')
    start_arg = DeclareLaunchArgument('start', default_value=start)
    display = LaunchConfiguration('display', default='true')
    display_arg = DeclareLaunchArgument('display', default_value=display)
    microphone_type = os.environ['MICROPHONE_TYPE']

    if compiled == 'True':      
        example_package_path = get_package_share_directory('example')
        xf_mic_asr_offline_package_path = get_package_share_directory('xf_mic_asr_offline')
    else:
        example_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/example'
        xf_mic_asr_offline_package_path = '/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/xf_mic_asr_offline'
    if microphone_type == 'xf':

        mic_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xf_mic_asr_offline_package_path, 'launch/mic_init.launch.py')),
        )
    else:
        mic_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(xf_mic_asr_offline_package_path, 'launch/wonderechopro_init.launch.py')),
        )

    color_sorting_node_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(example_package_path, 'example/opencv/color_sorting_node.launch.py')),
        launch_arguments={
            'broadcast': 'true',
        }.items(),
    )

    voice_control_color_sorting_node = Node(
        package='xf_mic_asr_offline',
        executable='voice_control_color_sorting.py',
        output='screen',
    )

    return [ 
            color_sorting_node_launch,
            mic_launch,
            voice_control_color_sorting_node,
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
