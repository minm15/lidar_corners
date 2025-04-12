#include "lidar_corners/Corners.h"
#include "lidar_corners/Utils.h"
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/eigen.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/common/intersections.h> 
#include <iostream>

static int iteration_count = 0;

bool getCorners(
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan, 
    int max_iters,
    pcl::PointCloud<pcl::PointXYZ>::Ptr edges_out
)
{
    std::cout << "[getCorners] Iteration: " << iteration_count << std::endl;

    if (!scan || scan->empty() || scan->size() < 10)
    {
        std::cerr << "[getCorners] insufficient point cloud" << std::endl;
        return false;
    }

    std::vector<Eigen::VectorXf> line_models;
    // RANSAC: find the four edges 
    for (int i = 0; i < 4; i++)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr sampled_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_inliers(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &pt : scan->points)
        {
            if (std::rand() % 3 == 0) 
                sampled_cloud->push_back(pt);
        }
        if (sampled_cloud->size() < 10)
        {
            std::cerr << "[getCorners] RANSAC fail, not enough points in sampled cloud." << std::endl;
            continue;
        }

        pcl::SampleConsensusModelLine<pcl::PointXYZ>::Ptr model_l(
            new pcl::SampleConsensusModelLine<pcl::PointXYZ>(sampled_cloud)
        );
        pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_l);
        ransac.setDistanceThreshold(0.01); 
        ransac.computeModel();

        std::vector<int> inliers;
        ransac.getInliers(inliers);

        Eigen::VectorXf model_coefficients;
        ransac.getModelCoefficients(model_coefficients);

        std::cout << "[getCorners] line found => " 
                  << model_coefficients.transpose() << std::endl;

        line_models.push_back(model_coefficients);

        pcl::copyPointCloud(*sampled_cloud, inliers, *final_inliers);
        edges_out->insert(edges_out->end(), final_inliers->begin(), final_inliers->end());
    }

    // Find the corner according to the line intersection
    pcl::PointCloud<pcl::PointXYZ>::Ptr corners(new pcl::PointCloud<pcl::PointXYZ>);
    Eigen::Vector4f p1, p2, intersection;
    for (int i = 0; i < 4; i++)
    {
        pcl::lineToLineSegment(
            line_models[i], 
            line_models[(i + 1) % 4], 
            p1, 
            p2
        );
        for (int j = 0; j < 3; j++)
        {
            intersection(j) = (p1(j) + p2(j)) / 2.0;
        }
        corners->push_back(
            pcl::PointXYZ(intersection(0), intersection(1), intersection(2))
        );
        std::cout << "[getCorners] Corner found: " 
                  << intersection.transpose() << std::endl;
    }

    edges_out->insert(edges_out->end(), corners->begin(), corners->end());

    iteration_count++;
    if (iteration_count >= max_iters)
    {
        std::cout << "Max iterations reached. Stopping process." << std::endl;
    }

    return true;
}
