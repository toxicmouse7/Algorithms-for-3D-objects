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
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/png_io.h>
#include <pcl/io/point_cloud_image_extractors.h>
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

#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>

#include <cmath>

#define PI 3.14159265

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

class CloudHandler {
public:
    template <class T>
    static T clamp(T value, T min, T max);
	void VoxelFilterCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr output);
    void RorFilterCloud(pcl::PointCloud<PointXYZ>::Ptr input, pcl::PointCloud<PointXYZ>::Ptr output);
	void RorFilterCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr output);
    pcl::ModelCoefficients::Ptr FindPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers);
	void CreateParallelepiped(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, PointXYZRGB);
    void CreateParallelepiped(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
	void CutCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr upper, pcl::PCLPointCloud2::Ptr lower);
    void TranslateToBase(pcl::PointCloud<PointXYZ>::Ptr input, pcl::PointCloud<PointXYZ>::Ptr parallelepiped, pcl::PointCloud<PointXYZ>::Ptr output);
    void ProjectOnXOY(pcl::PointCloud<PointXYZ>::Ptr input, pcl::PointCloud<PointXYZ>::Ptr output);
    void ExportImage(pcl::PointCloud<PointXYZ>::Ptr translated_cloud, bool flip, std::string filename);
    void RotateX(pcl::PointCloud<PointXYZ>::Ptr cloud, float angle, PointCloud<PointXYZ>::Ptr output);
    void RotateY(pcl::PointCloud<PointXYZ>::Ptr cloud, float angle, PointCloud<PointXYZ>::Ptr output);
    void RotateZ(pcl::PointCloud<PointXYZ>::Ptr cloud, float angle, PointCloud<PointXYZ>::Ptr output);
    void Scale(PointCloud<PointXYZ>::Ptr input, Eigen::Vector3f scaling, PointCloud<PointXYZ>::Ptr output);
    void Augmentation(pcl::PointCloud<PointXYZ>::Ptr cloud, std::string base_name);
    void Visualize(PointCloud<PointXYZ>::Ptr cloud, Eigen::Vector4f& mean, Eigen::Matrix3f& vects);
    void Visualize(PointCloud<PointXYZ>::Ptr cloud);
};
