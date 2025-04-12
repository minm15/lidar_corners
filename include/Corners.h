#ifndef LIDAR_CORNERS_CORNERS_H_
#define LIDAR_CORNERS_CORNERS_H_

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

bool getCorners(
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan, 
    int max_iters,
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges_out
);

#endif  
