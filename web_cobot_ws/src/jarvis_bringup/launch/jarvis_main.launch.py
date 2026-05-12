from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('jarvis_bringup')

    def launch(file):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg, 'launch', file))
        )

    return LaunchDescription([

        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),
        LogInfo(msg='  🤖 짱구네 ROS2 시스템 시작'),
        LogInfo(msg='━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'),

        # 1. 물체 감지 필터 (즉시)
        launch('object_filter.launch.py'),

        # 2. 제스처 제어 노드들 (1초 후)
        TimerAction(period=1.0, actions=[
            launch('gesture.launch.py'),
        ]),

        # 3. 음성 인식 노드 (2초 후)
        TimerAction(period=2.0, actions=[
            launch('voice.launch.py'),
        ]),

        TimerAction(period=3.0, actions=[
            launch('state_control.launch.py'),
        ]),

        LogInfo(msg='✅ 모든 노드 시작 완료'),

    ])
