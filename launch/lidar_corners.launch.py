from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lidar_corners',
            executable='lidar_corners_node',
            name='lidar_corners_node',
            parameters=[
                {'input_topic': '/velodyne_points'},
                {'z_max_for_ground': -0.35},
                {'ransac_dist_threshold': 0.1},
                {'normal_diff_threshold': 0.1},
                {'min_inliers': 30},
                {'max_ground_planes': 2},
            ],
            output='screen'
        ),
    ])
