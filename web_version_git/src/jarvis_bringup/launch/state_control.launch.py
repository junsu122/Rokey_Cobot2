from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='robot_state_control',
            executable='main_node',
            name='main_node',
            output='screen',
        ),

        Node(
            package='robot_state_control',
            executable='emergency_stop_node',
            name='emergency_stop_node',
            output='screen',
        ),

        Node(
            package='robot_state_control',
            executable='exception_manager_node',
            name='exception_manager_node',
            output='screen',
        ),

        Node(
            package='robot_state_control',
            executable='hand_recovery_node',
            name='hand_recovery_node',
            output='screen',
        ),

    ])
