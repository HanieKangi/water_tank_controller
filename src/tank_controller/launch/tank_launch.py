from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([

        Node(
            package='simulator',
            executable='tank_sim',
            name='simulator',
            output='screen'
        ),
    
        Node(
            package='tank_controller',
            executable='controller_node',
            name='tank_controller',
            output='screen'
        )
    ])