#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/passthrough.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

#include <cmath> 

using std::placeholders::_1;

class LidarPlaneExtractNode : public rclcpp::Node
{
public:
    LidarPlaneExtractNode()
    : Node("lidar_plane_extract_node")
    {
        // Subscribe the lidar topic
        this->declare_parameter<std::string>("input_topic", "/velodyne_points");
        std::string input_topic = this->get_parameter("input_topic").as_string();

        sub_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            input_topic,
            10,
            std::bind(&LidarPlaneExtractNode::cloudCallback, this, _1)
        );

        // Publish
        pub_plane_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "plane_points", 
            10
        );

        RCLCPP_INFO(this->get_logger(),
                    "LidarPlaneExtractNode started, subscribing to topic: %s",
                    input_topic.c_str());
    }

private:
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert from ROS to PCL
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *raw_cloud);

        if (raw_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "Received empty point cloud.");
            return;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr front_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        front_cloud->reserve(raw_cloud->size());
        for (auto &pt : raw_cloud->points)
        {
            float angle_deg = std::atan2(pt.y, pt.x) * 180.0f / static_cast<float>(M_PI);
            if (angle_deg >= -40.0f && angle_deg <= 40.0f)
            {
                front_cloud->push_back(pt);
            }
        }
        if (front_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "All points outside -30~30 deg. Nothing to process.");
            return;
        }

        // Remove the ground points
        pcl::PointCloud<pcl::PointXYZ>::Ptr no_ground_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        {
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(front_cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(-0.3, 100.0); 
            pass.filter(*no_ground_cloud);
        }
        if (no_ground_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "All points were filtered out as ground. No points left.");
            return;
        }

        // Use RANSAC to detect the plane
        int max_planes = 1;
        size_t min_inliers = 50;
        float distance_threshold = 0.4f;

        pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud(new pcl::PointCloud<pcl::PointXYZ>(*no_ground_cloud));
        pcl::PointCloud<pcl::PointXYZ>::Ptr planes_aggregate(new pcl::PointCloud<pcl::PointXYZ>);
        planes_aggregate->header   = remaining_cloud->header;
        planes_aggregate->is_dense = false;

        int plane_count = 0;
        while (plane_count < max_planes && !remaining_cloud->empty())
        {
            // a. RANSAC initialization
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(remaining_cloud)
            );
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
            ransac.setDistanceThreshold(distance_threshold);
            ransac.computeModel();

            // b. Obtain the inliers
            std::vector<int> inliers_indices;
            ransac.getInliers(inliers_indices);

            if (inliers_indices.size() < min_inliers)
            {
                RCLCPP_INFO(this->get_logger(),
                            "Plane [%d]: inliers too few (%zu). Stop extracting more planes.",
                            plane_count, inliers_indices.size());
                break;
            }

            pcl::PointCloud<pcl::PointXYZ>::Ptr this_plane(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::copyPointCloud(*remaining_cloud, inliers_indices, *this_plane);

            *planes_aggregate += *this_plane;

            // c. Remove the inliers for the current plane
            pcl::PointIndices::Ptr inliers_ptr(new pcl::PointIndices);
            inliers_ptr->indices = inliers_indices;
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(remaining_cloud);
            extract.setIndices(inliers_ptr);
            extract.setNegative(true);  
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*tmp);
            remaining_cloud.swap(tmp);

            RCLCPP_INFO(this->get_logger(),
                        "Plane [%d]: inliers=%zu. Remain points=%zu",
                        plane_count, inliers_indices.size(), remaining_cloud->size());

            plane_count++;
        }
        
        // Publish the result
        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(*planes_aggregate, out_msg);
        out_msg.header = msg->header; 
        pub_plane_->publish(out_msg);

        RCLCPP_INFO(this->get_logger(),
                    "Published [%d] planes, total inliers=%zu, remain=%zu",
                    plane_count, planes_aggregate->size(), remaining_cloud->size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_cloud_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_plane_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarPlaneExtractNode>());
    rclcpp::shutdown();
    return 0;
}
