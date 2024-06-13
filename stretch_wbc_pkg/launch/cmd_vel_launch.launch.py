import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='stretch_wbc_pkg',
            executable='sine_vel_pub',
            name='sine_vel_pub',
            output='screen',
            parameters=[{
                'frequency': 1.0,  
                'amplitude': 1.0   
            }]
        ),
    ])
