import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 실행 시 변경할 수 있는 변수(Argument) 설정 (선택 사항)
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='dsr01',
        description='Default Robot ID'
    )

    # 2. 노드 정의
    # Exception Manager 노드
    exception_manager_node = Node(
        package='robot_state_control',      # 패키지 이름
        executable='exception_manager_node', # setup.py에 정의한 entry_point 이름
        name='exception_manager_node',    # 노드 이름 (실행 시 변경 가능)
        output='screen',                  # 로그를 터미널에 출력
        parameters=[{'robot_id': LaunchConfiguration('robot_id')}] # 파라미터 전달
    )

    # Hand Recovery 노드
    hand_recovery_node = Node(
        package='robot_state_control',
        executable='hand_recovery_node',
        name='hand_recovery_node',
        output='screen'
    )

    emergency_stop_node = Node(
        package='robot_state_control',
        executable='emergency_stop_node',
        name='emergency_stop_node',
        # output='screen'
    )

    main_node = Node(
        package='robot_state_control',
        executable='main_node',
        name='main_node',
        # output='screen'
    )

    # 3. LaunchDescription에 노드들을 담아서 반환
    return LaunchDescription([
        robot_id_arg,
        exception_manager_node,
        hand_recovery_node,
        emergency_stop_node,
        main_node
    ])