#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <cmath> 
#include "lidar_corners_node.hpp"
#include "GroundRemoval.h"

using std::placeholders::_1;

class LidarPlaneExtractNode : public rclcpp::Node
{
public:
    LidarPlaneExtractNode()
    : Node("lidar_plane_extract_node")
    {
        // === Get Parameters ===
        this->declare_parameter<std::string>("lidar_topic", "/velodyne_points");
        this->declare_parameter<double>("z_max_for_ground", -0.35);
        this->declare_parameter<double>("ransac_dist_threshold", 0.1);
        this->declare_parameter<double>("normal_diff_threshold", 0.1);
        this->declare_parameter<int>("min_inliers", 30);
        this->declare_parameter<int>("max_ground_planes", 2);

        this->get_parameter("lidar_topic", lidar_topic);
        this->get_parameter("z_max_for_ground", z_max_for_ground_);
        this->get_parameter("ransac_dist_threshold", ransac_dist_threshold_);
        this->get_parameter("normal_diff_threshold", normal_diff_threshold_);
        this->get_parameter("min_inliers", min_inliers_);
        this->get_parameter("max_ground_planes", max_ground_planes_);

        // === Subscription & Publisher ===
        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic,
            10,
            std::bind(&LidarPlaneExtractNode::cloudCallback, this, _1)
        );

        pub_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "plane_points", 
            10
        );

        RCLCPP_INFO(this->get_logger(),
                    "LidarPlaneExtractNode started, subscribing to: %s",
                    lidar_topic.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // A. ROS -> PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw_cloud);
        if (raw_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty cloud.");
            return;
        }

        // B. Filter with angle
        pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        front_cloud->reserve(raw_cloud->size());
        for (auto &pt : raw_cloud->points)
        {
            float angle_deg = std::atan2(pt.y, pt.x) * 180.0f / static_cast<float>(M_PI);
            if (angle_deg >= -40.0f && angle_deg <= 40.0f)
                front_cloud->push_back(pt);
        }
        if (front_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "No points in [-40,40] deg range.");
            return;
        }

        // Remove the ground points
        float z_max_for_ground = -0.35f; 
        float ransac_dist_threshold = 0.1f; 
        float normal_diff_threshold = 0.1f; 
        size_t min_inliers = 30;   
        int max_ground_planes = 2; 

        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud = 
            advanced_ground_remove::removeGroundByRansacWithZ(
                front_cloud,
                static_cast<float>(z_max_for_ground_),
                static_cast<float>(ransac_dist_threshold_),
                static_cast<float>(normal_diff_threshold_),
                static_cast<size_t>(min_inliers_),
                max_ground_planes_
            );

        if (!no_ground_cloud || no_ground_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "All points removed or empty after ground removal.");
            return;
        }

        // Use RANSAC to detect the plane
        pcl::PointCloud<pcl::PointXYZ>::Ptr planes_aggregate(new pcl::PointCloud<pcl::PointXYZ>);
        planes_aggregate->header   = no_ground_cloud->header;
        planes_aggregate->is_dense = false;

        {
            // a. RANSAC initialization
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(no_ground_cloud)
            );
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
            ransac.setDistanceThreshold(0.3);  
            ransac.computeModel();
            
            // b. Obtain the inliers
            std::vector<int> inliers_idx;
            ransac.getInliers(inliers_idx);
            if (!inliers_idx.empty())
            {
                pcl::copyPointCloud(*no_ground_cloud, inliers_idx, *planes_aggregate);
            }
        }

        // Publish the result
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*planes_aggregate, out_msg);
        out_msg.header = msg->header; 
        pub_plane_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Publish plane with %zu points, remain cloud size=%zu",
                    planes_aggregate->size(), no_ground_cloud->size());
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPlaneExtractNode>());
    rclcpp::shutdown();
    return 0;
}
