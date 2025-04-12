#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <iostream>

int main(int argc, char **argv)
{
    if (argc < 2) {
        std::cerr << "Usage: detect_calib_board <input.pcd>" << std::endl;
        return -1;
    }

    // load the point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile(argv[1], *cloud) == -1) {
        PCL_ERROR("Couldn't read file\n");
        return -1;
    }
    std::cout << "Loaded " << cloud->points.size() << " points.\n";

    // 1. RANSAC to detect the plane
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setDistanceThreshold(0.01); // 1cm
    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.size() < 100) {
        std::cerr << "Could not find a large enough plane.\n";
        return -1;
    }

    std::cout << "Found plane: "
              << coefficients->values[0] << " "
              << coefficients->values[1] << " "
              << coefficients->values[2] << " "
              << coefficients->values[3] << std::endl;

    // 2. collect the pcd in the plane
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(false); 
    extract.filter(*plane_cloud);

    std::cout << "Extracted " << plane_cloud->size() << " plane points.\n";

    // 3. save the collected point cloud to a new file
    pcl::io::savePCDFileASCII("plane_points.pcd", *plane_cloud);
    std::cout << "Saved " << plane_cloud->size() << " plane points to plane_points.pcd\n";

    // 4. calculate Bounding Box
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(*plane_cloud, min_pt, max_pt);

    std::cout << "Bounding edges:\n";
    std::cout << min_pt.head<3>().transpose() << "\n";
    std::cout << max_pt.head<3>().transpose() << "\n";

    std::cout << "Corners:\n";
    std::cout << min_pt[0] << ", " << min_pt[1] << ", " << min_pt[2] << std::endl;
    std::cout << max_pt[0] << ", " << min_pt[1] << ", " << min_pt[2] << std::endl;
    std::cout << max_pt[0] << ", " << max_pt[1] << ", " << max_pt[2] << std::endl;
    std::cout << min_pt[0] << ", " << max_pt[1] << ", " << min_pt[2] << std::endl;

    return 0;
}
