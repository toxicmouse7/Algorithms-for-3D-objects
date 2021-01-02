#pragma once
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/bilateral.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>

#include <pcl/common/pca.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/integral_image_normal.h>

#include <boost/thread.hpp>
#include <boost/chrono/chrono.hpp>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

class CloudHandler {
public:
	void VoxelFilterCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr output);
	void RorFilterCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr output);
	void FindPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers);
	void CreateParallelepiped(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, PointXYZRGB);
    void CreateParallelepiped(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, PointXYZ);
	void CutCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr upper, pcl::PCLPointCloud2::Ptr lower);
};
