import os
from glob import glob
from setuptools import setup

package_name = 'example'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('example', '**/*.launch.py'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'resource'), glob(os.path.join('resource', '*.dae'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='2868673218@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bus_servo = example.simple.include.bus_servo_node:main',
            'led = example.simple.include.led_node:main',
            'buzzer = example.simple.include.buzzer_node:main',
            'fk = example.simple.include.fk:main',
            'ik = example.simple.include.ik:main',


            'camera_topic_invoke = example.opencv.include.camera_topic_invoke:main',
            'color_space = example.opencv.include.color_space:main',
            'color_threshold = example.opencv.include.color_threshold:main',
            'color_recognition = example.opencv.include.color_recognition:main',
            'pixel_coordinate_calculation = example.opencv.include.pixel_coordinate_calculation:main',
            'coordinate_system_transformation = example.opencv.include.coordinate_system_transformation:main',
            'object_attitude_calculation = example.opencv.include.object_attitude_calculation:main',
            'path_planning = example.opencv.include.path_planning:main',
            'positioning_clamp = example.opencv.include.positioning_clamp:main',
            'color_detect = example.opencv.include.color_detect_node:main',
            'color_sorting = example.opencv.include.color_sorting_node:main',
            'color_track = example.opencv.include.color_track_node:main',
            'tag_track = example.opencv.include.tag_track_node:main',
            'kcf_track = example.opencv.include.kcf_track_node:main',

            'yolov5_node = example.yolov5.yolov5_node:main',
            'yolov8_node = example.yolov8.yolov8_node:main',
            'waste_classification = example.yolov8.waste_classification:main',
            'face_mask = example.yolov8.face_mask:main',

            'face_mesh = example.mediapipe.include.face_mesh:main',
            'face_tracking = example.mediapipe.include.face_tracking:main',
            'finger_trajectory = example.mediapipe.include.finger_trajectory:main',
            'hand_gesture = example.mediapipe.include.hand_gesture:main',
            'mankind_pose = example.mediapipe.include.mankind_pose:main',


            'get_depth_rgb_img = example.rgbd_function.include.get_depth_rgb_img:main',
            'distance_measure = example.rgbd_function.include.distance_measure:main',
            'rgb_depth_to_pointcloud = example.rgbd_function.include.rgb_depth_to_pointcloud:main',
            'remove_too_high = example.rgbd_function.include.remove_too_high:main',
            'track_and_grab = example.rgbd_function.include.track_and_grab:main',
            'shape_recognition = example.rgbd_function.include.shape_recognition:main',
        ],
    },
)
