import os
from launch_ros.actions import Node
from launch import LaunchDescription, LaunchService
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction

def launch_setup(context):
    appid = LaunchConfiguration('appid', default="1c24f60f")
    enable_setting = LaunchConfiguration('enable_setting', default='false')
    confidence = LaunchConfiguration('confidence', default='18')
    seconds_per_order = LaunchConfiguration('seconds_per_order', default='15')
    chinese_awake_words = LaunchConfiguration('chinese_awake_words', default='xiao3 huan4 xiao3 huan4')
    english_awake_words = LaunchConfiguration('english_awake_words', default='hello hi wonder')
    language = LaunchConfiguration('language', default=os.environ['ASR_LANGUAGE']).perform(context)

    # NEW
    port = LaunchConfiguration('port', default='/dev/ring_mic')

    appid_arg = DeclareLaunchArgument('appid', default_value=appid)
    enable_setting_arg = DeclareLaunchArgument('enable_setting', default_value=enable_setting)
    confidencee_arg = DeclareLaunchArgument('confidence', default_value=confidence)
    seconds_per_order_arg = DeclareLaunchArgument('seconds_per_order', default_value=seconds_per_order)
    chinese_awake_words_arg = DeclareLaunchArgument('chinese_awake_words', default_value=chinese_awake_words)
    english_awake_words_arg = DeclareLaunchArgument('english_awake_words', default_value=english_awake_words)
    language_arg = DeclareLaunchArgument('language', default_value=language)

    # NEW
    port_arg = DeclareLaunchArgument(
        'port',
        default_value=port
    )

    if language == 'Chinese':
        awake_words = chinese_awake_words
    else:
        awake_words = english_awake_words

    awake_node = Node(
        package="xf_mic_asr_offline",
        executable="awake_node.py",
        output='screen',
        parameters=[{
            "port": port,          # <-- changed
            "mic_type": "mic6_circle",
            "awake_word": awake_words,
            "enable_setting": enable_setting
        }],
    )

    asr_node = Node(
        package="xf_mic_asr_offline",
        executable="asr_node.py",
        output='screen',
        parameters=[{
            "confidence": confidence,
            "seconds_per_order": seconds_per_order
        }],
    )

    voice_control = Node(
        package="xf_mic_asr_offline",
        executable="voice_control",
        output='screen',
        parameters=[{
            "appid": appid,
            "source_path": "/home/hiwonder/jetarm_ros2_ws/src/JetArm/src/xf_mic_asr_offline"
        }],
    )

    return [
        appid_arg,
        enable_setting_arg,
        confidencee_arg,
        seconds_per_order_arg,
        chinese_awake_words_arg,
        english_awake_words_arg,
        language_arg,
        port_arg,          # <-- NEW
        awake_node,
        voice_control,
        asr_node,
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])


if __name__ == '__main__':
    ld = generate_launch_description()

    ls = LaunchService()
    ls.include_launch_description(ld)
    ls.run()
