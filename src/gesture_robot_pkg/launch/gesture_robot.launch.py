"""
gesture_robot.launch.py
────────────────────────
4개 노드를 한 번에 실행하는 런치 파일.

사용법:
    ros2 launch gesture_robot_pkg gesture_robot.launch.py

파라미터 오버라이드 예시:
    ros2 launch gesture_robot_pkg gesture_robot.launch.py \\
        webcam_index:=4 position_control:=false yolo_conf:=0.4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Launch Arguments ────────────────────────────
    webcam_index_arg = DeclareLaunchArgument(
        'webcam_index', default_value='0',
        description='Webcam device index (cv2.VideoCapture)')

    position_control_arg = DeclareLaunchArgument(
        'position_control', default_value='true',
        description='True=space-mapping position control, False=velocity control')

    yolo_model_arg = DeclareLaunchArgument(
        'yolo_model',
        default_value=PathJoinSubstitution([
            FindPackageShare('gesture_robot_pkg'), 'data', '5.8best.pt'
        ]),
        description='Path to YOLOv8 weights file')

    yolo_conf_arg = DeclareLaunchArgument(
        'yolo_conf', default_value='0.5',
        description='YOLO confidence threshold')

    hover_sec_arg = DeclareLaunchArgument(
        'hover_sec', default_value='2.0',
        description='Hover duration (seconds) to trigger object selection')

    # ── Nodes ────────────────────────────────────────

    webcam_teleop = Node(
        package='gesture_robot_pkg',
        executable='webcam_teleop_node',
        name='webcam_teleop_node',
        output='screen',
        parameters=[{
            'webcam_index':     LaunchConfiguration('webcam_index'),
            'position_control': LaunchConfiguration('position_control'),
        }],
    )

    vision = Node(
        package='gesture_robot_pkg',
        executable='vision_node',
        name='vision_node',
        output='screen',
        parameters=[{
            'yolo_model': LaunchConfiguration('yolo_model'),
            'yolo_conf':  LaunchConfiguration('yolo_conf'),
            'hover_sec':  LaunchConfiguration('hover_sec'),
        }],
    )

    orchestrator = Node(
        package='gesture_robot_pkg',
        executable='orchestrator_node',
        name='orchestrator_node',
        output='screen',
    )

    pick_and_place = Node(
        package='gesture_robot_pkg',
        executable='pick_and_place_node',
        name='pick_and_place_node',
        output='screen',
    )

    return LaunchDescription([
        webcam_index_arg,
        position_control_arg,
        yolo_model_arg,
        yolo_conf_arg,
        hover_sec_arg,
        LogInfo(msg='[gesture_robot] Launching 4 nodes...'),
        webcam_teleop,
        vision,
        orchestrator,
        pick_and_place,
    ])
