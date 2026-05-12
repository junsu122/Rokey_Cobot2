from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='gesture_robot_pkg',
            executable='vision_node',
            name='vision_node',
            output='screen',
        ),

        Node(
            package='gesture_robot_pkg',
            executable='pick_and_place_node',
            name='pick_and_place_node',
            output='screen',
        ),

        Node(
            package='gesture_robot_pkg',
            executable='orchestrator_node',
            name='orchestrator_node',
            output='screen',
        ),

    ])
