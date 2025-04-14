#ifndef LIDAR_PLANE_EXTRACT_NODE_HPP_
#define LIDAR_PLANE_EXTRACT_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include "GroundRemoval.h"

class LidarPlaneExtractNode : public rclcpp::Node
{
public:
    LidarPlaneExtractNode();

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_;

    std::string lidar_topic;
    double z_max_for_ground_;
    double ransac_dist_threshold_;
    double normal_diff_threshold_;
    int min_inliers_;
    int max_ground_planes_;
    double dist_limit_;
};

#endif  // LIDAR_PLANE_EXTRACT_NODE_HPP_
