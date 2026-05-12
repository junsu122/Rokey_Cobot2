from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'robot_state_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junsu',
    maintainer_email='junsoo122@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    # extras_require={
    #     'test': [
    #         'pytest',
    #     ],
    # },
    entry_points={
        'console_scripts': [

######################################################## 메인코드 ##################################
            'main_node = robot_state_control.main_node:main',
            'emergency_stop_node = robot_state_control.emergency_stop_node:main',
            'exception_manager_node = robot_state_control.exception_manager_node:main',
            'hand_recovery_node = robot_state_control.hand_recovery:main',
##############3 ######################################### 테스트코드 ################################

            'test_status_sub = robot_state_control._test_status_sub:main',

        ],
    },
)
