#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace advanced_ground_remove
{
    /**
     * @brief Filter by Z value + multiple rounds of RANSAC for ground removal.
     *  Workflow:
     *   1) Make a shallow copy of in_cloud into all_cloud
     *   2) Filter points with z < max_z_for_ground from all_cloud to form candidate_cloud
     *   3) Perform RANSAC on candidate_cloud multiple times. If a plane is "approximately horizontal", treat it as ground and remove it from candidate_cloud
     *   4) Remove the ground inliers (from candidate_cloud) from all_cloud as well
     *   5) Return the modified all_cloud (i.e., global cloud minus ground)
     *
     * @param in_cloud              Input point cloud
     * @param max_z_for_ground      Only points with z < max_z_for_ground are considered for ground RANSAC
     * @param distance_threshold    RANSAC inlier distance threshold
     * @param normal_diff_threshold Angular threshold (in radians) between plane normal and (0,0,1); smaller values mean closer to horizontal
     * @param min_inliers           Planes with inliers less than this are not considered ground
     * @param max_ground_planes     Maximum number of ground planes to detect
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr The cloud after ground removal (includes z > max_z_for_ground and non-ground points)
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundByRansacWithZ(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
        float max_z_for_ground,
        float distance_threshold,
        float normal_diff_threshold,
        size_t min_inliers,
        int max_ground_planes
    );
}
