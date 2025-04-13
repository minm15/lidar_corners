#include "GroundRemoval.h"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/filters/passthrough.h>
#include <Eigen/Dense>
#include <cmath> 

namespace advanced_ground_remove
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundByRansacWithZ(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
        float max_z_for_ground,
        float distance_threshold,
        float normal_diff_threshold,
        size_t min_inliers,
        int max_ground_planes
    )
    {
        if (!in_cloud || in_cloud->empty())
        {
            return pcl::PointCloud<pcl::PointXYZ>::Ptr(
                new pcl::PointCloud<pcl::PointXYZ>(*in_cloud)
            );
        }

        // 1. Copy the whole point cloud, will be removed later
        pcl::PointCloud<pcl::PointXYZ>::Ptr all_cloud(
            new pcl::PointCloud<pcl::PointXYZ>(*in_cloud));

        // 2. Filter to get candidate points with z < max_z_for_ground
        pcl::PointCloud<pcl::PointXYZ>::Ptr candidate_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        candidate_cloud->reserve(all_cloud->size());
        for (auto &pt : all_cloud->points)
        {
            if (pt.z < max_z_for_ground)
            {
                candidate_cloud->push_back(pt);
            }
        }

        if (candidate_cloud->empty())
        {
            return all_cloud;
        }

        // 3. Perform RANSAC on candidate_cloud
        int ground_plane_count = 0;

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_for_ransac(
            new pcl::PointCloud<pcl::PointXYZ>(*candidate_cloud)
        );

        while (ground_plane_count < max_ground_planes && !cloud_for_ransac->empty())
        {
            // A. Perform RANSAC
            pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(
                new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(cloud_for_ransac)
            );
            pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);
            ransac.setDistanceThreshold(distance_threshold);
            ransac.computeModel();

            std::vector<int> inliers_idx;
            ransac.getInliers(inliers_idx);

            if (inliers_idx.size() < min_inliers)
            {
                break;
            }

            // B. Get plane normal vector
            Eigen::VectorXf coeffs;
            ransac.getModelCoefficients(coeffs);
            Eigen::Vector3f plane_normal(coeffs[0], coeffs[1], coeffs[2]);
            plane_normal.normalize();

            float angle = std::acos(std::fabs(
                plane_normal.dot(Eigen::Vector3f(0.f, 0.f, 1.f))
            ));

            if (angle > normal_diff_threshold)
            {
                break;
            }

            // C. Remove ground points indicated by inliers_idx
            pcl::PointIndices::Ptr ground_inliers(new pcl::PointIndices);
            ground_inliers->indices = inliers_idx;
            
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            extract.setInputCloud(cloud_for_ransac);
            extract.setIndices(ground_inliers);
            extract.setNegative(true);
            pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
            extract.filter(*tmp);
            cloud_for_ransac.swap(tmp);

            for (auto idx : inliers_idx)
            {
                if (idx < 0 || idx >= static_cast<int>(candidate_cloud->size()))
                    continue;
                const auto &ground_pt = candidate_cloud->points[idx];

                for (size_t k = 0; k < all_cloud->size(); ++k)
                {
                    auto &ptA = all_cloud->points[k];
                    if (std::fabs(ptA.x - ground_pt.x) < 1e-5 &&
                        std::fabs(ptA.y - ground_pt.y) < 1e-5 &&
                        std::fabs(ptA.z - ground_pt.z) < 1e-5 )
                    {
                        all_cloud->points[k] = all_cloud->points.back();
                        all_cloud->points.pop_back();
                        break;
                    }
                }
            }

            ground_plane_count++;
        }

        return all_cloud;
    }
}
