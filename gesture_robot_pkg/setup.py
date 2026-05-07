from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gesture_robot_pkg'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
         [f'resource/{package_name}']),
        (f'share/{package_name}', ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),
         (os.path.join('share', package_name, 'data'),
         glob('data/*')),   
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='Gesture robot control nodes',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_teleop_node  = gesture_robot_pkg.webcam_teleop_node:main',
            'vision_node         = gesture_robot_pkg.vision_node:main',
            'orchestrator_node   = gesture_robot_pkg.orchestrator_node:main',
            'pick_and_place_node = gesture_robot_pkg.pick_and_place_node:main',
        ],
    },
)
