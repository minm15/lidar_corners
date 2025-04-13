cmake_minimum_required(VERSION 3.8)
project(lidar_corners)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io sample_consensus)

include_directories(
  include
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# === lidar_corners_node ===
add_executable(lidar_corners_node
  src/lidar_corners_node.cpp
  src/Corners.cpp
  src/GroundRemoval.cpp
)

# === detect_calib_board ===
add_executable(detect_calib_board
  src/detect_calib_board.cpp
)

# === Link common dependencies ===
foreach(target IN ITEMS lidar_corners_node detect_calib_board)
  target_link_libraries(${target}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
  )
  ament_target_dependencies(${target}
    rclcpp
    pcl_conversions
    sensor_msgs
  )
endforeach()

# === Install both targets ===
install(TARGETS
  lidar_corners_node
  detect_calib_board
  DESTINATION lib/${PROJECT_NAME}
)

# === Install headers (if needed) ===
install(DIRECTORY include/
  DESTINATION include/
)

# === Install launch files ===
install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
