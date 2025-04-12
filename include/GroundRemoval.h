#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace ground_remove_utils
{
    /**
     * @brief 根據 RANSAC 與法向量約束，去除地面平面 (或接近水平的最大平面)
     * 
     * @param in_cloud               輸入原始點雲
     * @param distance_threshold     RANSAC inlier距離閾值
     * @param normal_diff_threshold  判斷是否「朝上」的角度閾值(單位: 弧度); 例如 0.1 大約是 ~5.7度
     * @param min_inliers            若 inlier 數量小於這個值，不認定為地面
     * @return pcl::PointCloud<pcl::PointXYZ>::Ptr 移除地面後的點雲
     */
    pcl::PointCloud<pcl::PointXYZ>::Ptr removeGroundPoints(
       const pcl::PointCloud<pcl::PointXYZ>::Ptr &in_cloud,
       float distance_threshold,
       float normal_diff_threshold,
       size_t min_inliers
    );
}
