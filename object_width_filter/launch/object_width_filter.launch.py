from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 노드 1: 아까 만든 각도 퍼블리셔 예시
        Node(
            package='object_width_filter',  # 패키지 이름
            executable='no_ground', # setup.py에 등록된 실행 파일 이름
            # name='advanced_ground_remover',     # 실행될 때 노드 이름 (선택)
            # parameters=[{'target_angle': 45.0}], # 파라미터 설정 (선택)
            output='screen'               # 로그 출력 설정
        ),
        
        # 노드 2: 수신 대기할 노드나 다른 로봇 제어 노드
        Node(
            package='object_width_filter',
            executable='z_normalization',
            # name='listener_node',
            # remappings=[('/old_topic', '/object_angle')], # 토픽 이름 변경 (선택)
            output='screen'
        ),

        Node(
            package='object_width_filter',  # 패키지 이름
            executable='length_filter', # setup.py에 등록된 실행 파일 이름
            # name='advanced_ground_remover',     # 실행될 때 노드 이름 (선택)
            output='screen'               # 로그 출력 설정
        ),

        Node(
            package='object_width_filter',  # 패키지 이름
            executable='publish_angle', # setup.py에 등록된 실행 파일 이름
            # name='advanced_ground_remover',     # 실행될 때 노드 이름 (선택)
            output='screen'               # 로그 출력 설정
        ),
    ])