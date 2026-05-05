from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'object_width_filter'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 아래 줄을 추가하여 launch 폴더의 모든 .py 파일을 설치 경로로 복사
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junsu',
    maintainer_email='junsoo122@naver.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
############################################ 런치 실행 코드 ############################################
            
## 메인 실행 코드 ---->                   object_width_filter.launch.py

############################################ 개별 실행 코드 ############################################

            'length_filter = object_width_filter.length_filter:main',
            'no_ground = object_width_filter.no_ground:main',
            'publish_angle = object_width_filter.publish_angle:main',
            'z_normalization = object_width_filter.z_normalization:main',

############################################## 테스트 코드 #############################################

            'test_angle = object_width_filter._test_angle_test_publisher:main',
            'test_joint = object_width_filter._test_joint6_check:main',
            'test_cam = object_width_filter._test_cam2depth:main',
        ],
    },
)
