from glob import glob
import os

from setuptools import find_packages
from setuptools import setup


package_name = 'ventuno_face_tracker'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(
        exclude=['test']
    ),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join(
                'share',
                package_name,
                'launch'
            ),
            glob('launch/*.launch.py')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hiwonder',
    maintainer_email='support@example.com',
    description=(
        'Direct-servo MediaPipe face tracker '
        'for Ventuno Q and JetArm'
    ),
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            (
                'face_tracker = '
                'ventuno_face_tracker.'
                'face_tracker_node:main'
            ),
        ],
    },
)
