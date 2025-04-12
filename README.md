# Lidar Corners

## Build
```
colcon build --packages-select lidar_corners
source install/setup.bash
```

## Usage
This is the core ROS 2 node for future integration with LiDAR topics (e.g., /velodyne_points). It is currently under development.
```
ros2 launch lidar_corners lidar_corners.launch.py
```
## Development Progress 

This project is under active development. The current goal is to extract and visualize corner-like features from LiDAR point clouds. Below is a short demo of the current processing pipeline:

![Demo](assets/demo.gif)

### Features in Progress

- Write an advanced algorithm to remove the ground point rather than filtering with z-value
- Filter out noisy planes



