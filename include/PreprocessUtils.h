// PreprocessUtil.h
#pragma once

#include <opencv2/opencv.hpp>           
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>   
#include <Eigen/Core>
#include <Eigen/Geometry>

struct myPointXYZRID
{
	PCL_ADD_POINT4D;
	float intensity;
	uint16_t ring;
	float range;
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(myPointXYZRID,
	(float, x, x)
	(float, y, y)
	(float, z, z)
	(float, intensity, intensity)
	(uint16_t, ring, ring)
	(float, range, range)
)

struct MyConfig
{
	cv::Size image_size;            
	float intensity_thresh;         
	int num_of_markers;            
	int max_iters;
	cv::Mat P;                      
	std::vector<float> initialTra;  
};


inline pcl::PointCloud<pcl::PointXYZ> toPointsXYZ(const pcl::PointCloud<myPointXYZRID> &cloud_in)
{
	pcl::PointCloud<pcl::PointXYZ> out;
	out.header   = cloud_in.header;
	out.width    = cloud_in.width;
	out.height   = cloud_in.height;
	out.is_dense = cloud_in.is_dense;
	out.reserve(cloud_in.size());

	for (const auto &pt : cloud_in)
		out.push_back(pcl::PointXYZ(pt.x, pt.y, pt.z));

	return out;
}

inline pcl::PointCloud<myPointXYZRID> transform(
	const pcl::PointCloud<myPointXYZRID> &pc,
	float x, float y, float z,
	float rot_x, float rot_y, float rot_z)
{
	Eigen::Affine3f tf = pcl::getTransformation(x, y, z, rot_x, rot_y, rot_z);

	pcl::PointCloud<myPointXYZRID> out;
	out.header   = pc.header;
	out.width    = pc.width;
	out.height   = pc.height;
	out.is_dense = pc.is_dense;
	out.reserve(pc.size());

	for (const auto &pt : pc)
	{
		Eigen::Vector3f orig(pt.x, pt.y, pt.z);
		Eigen::Vector3f transformed = tf * orig;

		myPointXYZRID new_pt;
		new_pt.x = transformed.x();
		new_pt.y = transformed.y();
		new_pt.z = transformed.z();
		new_pt.intensity = pt.intensity;
		new_pt.ring      = pt.ring;
		new_pt.range     = pt.range;
		out.push_back(new_pt);
	}
	return out;
}
