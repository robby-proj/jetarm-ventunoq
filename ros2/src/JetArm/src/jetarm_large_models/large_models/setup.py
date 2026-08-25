import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'large_models'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('large_models', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='1270161395@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vocal_detect = large_models.vocal_detect:main',
            'agent_process = large_models.agent_process:main',
            'tts_node = large_models.tts_node:main',
            'object_transport = large_models.object_transport:main',
            'track_anything = large_models.track_anything:main',
            'intelligent_grasp = large_models.intelligent_grasp:main',
            'llm_control_servo = large_models.llm_control_servo:main',
            'llm_object_sorting = large_models.llm_object_sorting:main',
            'llm_waste_classification = large_models.llm_waste_classification:main',
            'llm_3d_object_sorting = large_models.llm_3d_object_sorting:main',
            'vllm_with_camera = large_models.vllm_with_camera:main',
            'vllm_track = large_models.vllm_track:main',
            'vllm_dietitianl = large_models.vllm_dietitianl:main',
            'vllm_object_transport = large_models.vllm_object_transport:main',
        ],
    },
)
