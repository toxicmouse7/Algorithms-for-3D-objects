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
using namespace Eigen;

class CloudHandler {
public:
    template <class T>
    static T clamp(T value, T min, T max);
	void VoxelFilterCloud(PCLPointCloud2::Ptr input, PCLPointCloud2::Ptr output);
    void RorFilterCloud(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output);
	void RorFilterCloud(PCLPointCloud2::Ptr input, PCLPointCloud2::Ptr output);
    ModelCoefficients::Ptr FindPlane(PointCloud<PointXYZRGB>::Ptr cloud, PointIndices::Ptr inliers);
	void CreateParallelepiped(PointCloud<PointXYZRGB>::Ptr input, PointCloud<PointXYZRGB>::Ptr output, PointXYZRGB);
    void CreateParallelepiped(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output);
	void CutCloud(PCLPointCloud2::Ptr input, PCLPointCloud2::Ptr upper, PCLPointCloud2::Ptr lower);
    void TranslateToBase(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr parallelepiped, PointCloud<PointXYZ>::Ptr output);
    void ProjectOnXOY(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output);
    void ExportImage(PointCloud<PointXYZ>::Ptr translated_cloud, std::string filename);
    void RotateX(PointCloud<PointXYZ>::Ptr cloud, float angle, PointCloud<PointXYZ>::Ptr output);
    void RotateY(PointCloud<PointXYZ>::Ptr cloud, float angle, PointCloud<PointXYZ>::Ptr output);
    void RotateZ(PointCloud<PointXYZ>::Ptr cloud, float angle, PointCloud<PointXYZ>::Ptr output);
    void Scale(PointCloud<PointXYZ>::Ptr input, Vector3f scaling, PointCloud<PointXYZ>::Ptr output);
    void FlipX(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output);
    void Augmentation(PointCloud<PointXYZ>::Ptr cloud, std::string base_name, bool flip);
    void Visualize(PointCloud<PointXYZ>::Ptr cloud, Eigen::Vector4f& mean, Eigen::Matrix3f& vects);
    void Visualize(PointCloud<PointXYZ>::Ptr cloud);
};
