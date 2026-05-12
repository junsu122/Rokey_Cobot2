from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='jarvis_voice_pkg',
            executable='voice_intent_node',
            name='voice_intent_node',
            output='screen',
        ),

        # workspace_scan_node - 개발 홀딩 중, 완성 후 아래 주석 해제
        # Node(
        #     package='jarvis_voice_pkg',
        #     executable='workspace_scan_node',
        #     name='workspace_scan_node',
        #     output='screen',
        # ),

    ])
