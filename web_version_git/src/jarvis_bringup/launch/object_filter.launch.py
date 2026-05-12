from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='object_width_filter',
            executable='no_ground',
            name='no_ground',
            output='screen',
        ),

        Node(
            package='object_width_filter',
            executable='z_normalization',
            name='z_normalization',
            output='screen',
        ),

        Node(
            package='object_width_filter',
            executable='length_filter',
            name='length_filter',
            output='screen',
        ),

        Node(
            package='object_width_filter',
            executable='publish_angle',
            name='publish_angle',
            output='screen',
        ),

        Node(
            package='object_width_filter',
            executable='vision_node2publish_angle',
            name='vision_node2publish_angle',
            output='screen',
        ),

    ])
