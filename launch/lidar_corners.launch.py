import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_corners',           
            executable='lidar_corners_node',    
            name='lidar_corners_node',         
            parameters=[
                {'lidar_topic': '/velodyne_points'} 
            ],
            output='screen'
        ),
    ])
