# Lidar Corners

## Build
```
colcon build --packages-select lidar_corners
source install/setup.bash
```

## Usage
```
ros2 run lidar_corners lidar_corners_node
ros2 launch lidar_corners lidar_corners.launch.py
```
This is the core ROS 2 node for future integration with LiDAR topics (e.g., /velodyne_points). It is currently under development.

```
ros2 run lidar_corners detect_calib_board <path_to_file.pcd>
```
This tool takes a single .pcd file, detects the planar surface using RANSAC, and estimates the bounding box of the detected plane.



