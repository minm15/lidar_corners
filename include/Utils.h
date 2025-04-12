// Utils.h
#ifndef LIDAR_CORNERS_UTILS_H_
#define LIDAR_CORNERS_UTILS_H_

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>

inline cv::Point project(const pcl::PointXYZ &pt, const cv::Mat &projection_matrix)
{
    cv::Mat pt_3D = (cv::Mat_<float>(4, 1) << pt.x, pt.y, pt.z, 1.0);

    cv::Mat pt_2D = projection_matrix * pt_3D;
    
    float w = pt_2D.at<float>(2);
    if (std::fabs(w) < 1e-5)  
        return cv::Point(-9999, -9999);

    int x = static_cast<int>(pt_2D.at<float>(0) / w);
    int y = static_cast<int>(pt_2D.at<float>(1) / w);

    return cv::Point(x, y);
}

#endif  // LIDAR_CORNERS_UTILS_H_
