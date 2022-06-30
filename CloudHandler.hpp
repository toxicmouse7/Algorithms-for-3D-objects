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

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace Eigen;

class CloudHandler
{
private:
    template<class PointT>
    static auto x_comp(const PointT& p1, const PointT& p2) { return p1.x < p2.x; };

    template<class PointT>
    static auto y_comp(const PointT& p1, const PointT& p2) { return p1.y < p2.y; };

    template<class PointT>
    static auto z_comp(const PointT& p1, const PointT& p2) { return p1.z < p2.z; };

public:
    template<class T>
    static T clamp(T value, T min, T max)
    {
        return std::max(std::min(value, max), min);
    }

    static void VoxelFilterCloud(const PCLPointCloud2::Ptr& input,
                                 const PCLPointCloud2::Ptr& output);

    template<class PointT>
    static void RorFilterCloud(const typename PointCloud<PointT>::Ptr& input,
                               const typename PointCloud<PointT>::Ptr& output)
    {
        RadiusOutlierRemoval<PointT> ror;
        ror.setRadiusSearch(0.025);
        ror.setMinNeighborsInRadius(10);
        ror.setInputCloud(input);
        ror.filter(*output);
    }

    static void RorFilterCloud(const PCLPointCloud2::Ptr& input,
                               const PCLPointCloud2::Ptr& output);

    template<class PointT>
    static ModelCoefficients::Ptr FindPlane(const typename PointCloud<PointT>::Ptr& cloud,
                                            const PointIndices::Ptr& inliers)
    {
        // Find grownd plane
        ModelCoefficients::Ptr coefficients(new ModelCoefficients);
        // Create the segmentation object
        SACSegmentation<PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients(true);
        // Mandatory
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_LMEDS);
        /*
        const static int SAC_RANSAC  = 0;
        const static int SAC_LMEDS   = 1;
        const static int SAC_MSAC    = 2;
        const static int SAC_RRANSAC = 3;
        const static int SAC_RMSAC   = 4;
        const static int SAC_MLESAC  = 5;
        const static int SAC_PROSAC  = 6;
        */
        seg.setMaxIterations(1000);
        seg.setDistanceThreshold(0.04);


        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

        if (inliers->indices.empty())
        {
            PCL_ERROR("Could not estimate a planar model for the given dataset.");
        }

        std::cerr << "Model coefficients: " << coefficients->values[0] << " "
                  << coefficients->values[1] << " "
                  << coefficients->values[2] << " "
                  << coefficients->values[3] << std::endl;


        std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;


        for (int index: inliers->indices)
        {
            cloud->points[index].r = 0;
            cloud->points[index].g = 255;
            cloud->points[index].b = 0;
            cloud->points[index].a = 255;
        }

        for (int i = 0; i < cloud->points.size(); i++)
            if (cloud->points[i].g != 255)
            {
                cloud->erase(cloud->begin() + i, cloud->begin() + i + 1);
                i--;
            }

        return coefficients;
    };

    template<class PointT>
    static void CreateParallelepiped(const typename PointCloud<PointT>::Ptr& input,
                                     const typename PointCloud<PointT>::Ptr& output)
    {
        PointT coloredPoint;

        float xMin = std::min_element(input->begin(), input->end(), CloudHandler::x_comp<PointT>)->x;
        float xMax = std::max_element(input->begin(), input->end(), CloudHandler::x_comp<PointT>)->x;
        float yMin = std::min_element(input->begin(), input->end(), CloudHandler::y_comp<PointT>)->y;
        float yMax = std::max_element(input->begin(), input->end(), CloudHandler::y_comp<PointT>)->y;
        float zMin = std::min_element(input->begin(), input->end(), CloudHandler::z_comp<PointT>)->z;
        float zMax = std::max_element(input->begin(), input->end(), CloudHandler::z_comp<PointT>)->z;

        // create parallelepiped around cloud
        for (float x = xMin; x < xMax; x += 0.01)
        {
            coloredPoint.x = x;
            coloredPoint.y = yMin;
            coloredPoint.z = zMin;
            output->push_back(coloredPoint);
            coloredPoint.y = yMax;
            output->push_back(coloredPoint);
            coloredPoint.z = zMax;
            output->push_back(coloredPoint);
            coloredPoint.y = yMin;
            output->push_back(coloredPoint);
        }
        for (float y = yMin; y < yMax; y += 0.01)
        {
            coloredPoint.x = xMin;
            coloredPoint.y = y;
            coloredPoint.z = zMin;
            output->push_back(coloredPoint);
            coloredPoint.x = xMax;
            output->push_back(coloredPoint);
            coloredPoint.z = zMax;
            output->push_back(coloredPoint);
            coloredPoint.x = xMin;
            output->push_back(coloredPoint);
        }
        for (float z = zMin; z < zMax; z += 0.01)
        {
            coloredPoint.x = xMin;
            coloredPoint.y = yMin;
            coloredPoint.z = z;
            output->push_back(coloredPoint);
            coloredPoint.x = xMax;
            output->push_back(coloredPoint);
            coloredPoint.y = yMax;
            output->push_back(coloredPoint);
            coloredPoint.x = xMin;
            output->push_back(coloredPoint);
        }
    }


    static void CutCloud(const PCLPointCloud2::Ptr& input,
                         const PCLPointCloud2::Ptr& upper,
                         const PCLPointCloud2::Ptr& lower);

    template<class PointT>
    static void TranslateToBase(const typename PointCloud<PointT>::Ptr& input,
                                const typename PointCloud<PointT>::Ptr& parallelepiped,
                                const typename PointCloud<PointT>::Ptr& output)
    {
        float xMin = parallelepiped->begin()->x;
        float yMin = parallelepiped->begin()->y;
        float zMin = parallelepiped->begin()->z;

        for (auto& i: *parallelepiped)
        {
            xMin = std::min(xMin, i.x);
            yMin = std::min(yMin, i.y);
            zMin = std::min(zMin, i.z);
        }

        Eigen::Matrix4f first_translation(4, 4);
        first_translation << 1, 0, 0, -xMin,
                0, 1, 0, -yMin,
                0, 0, 1, -zMin,
                0, 0, 0, 1;
        pcl::transformPointCloud(*input, *output, first_translation);
    };

    template<class PointT>
    static void ProjectOnXOY(const typename PointCloud<PointT>::Ptr& input,
                             const typename PointCloud<PointT>::Ptr& output)
    {
        //might be used for auto-translating
        ModelCoefficients::Ptr coefficients(new ModelCoefficients);
        coefficients->values.resize(4);
        coefficients->values[0] = coefficients->values[1] = 0;
        coefficients->values[2] = 1;
        coefficients->values[3] = 1;

        pcl::ProjectInliers<PointT> proj;
        proj.setModelType(pcl::SACMODEL_PLANE);
        proj.setInputCloud(input);
        proj.setModelCoefficients(coefficients);
        proj.filter(*output);
        std::cout << output->points.size() << " - points in projection" << std::endl;
        std::cout << input->points.size() << " - points in original" << std::endl;
    }

    template<class PointT>
    static void ExportImageDepth(const typename PointCloud<PointT>::Ptr& translated_cloud,
                                 std::string& filename)
    {
        cv::Size img_size(299, 150);
        cv::Mat image = cv::Mat::zeros(img_size, CV_8UC1);

        auto z_comp = [](const PointT& p1, const PointT& p2) { return p1.z < p2.z; };

        float zMax = std::max_element(translated_cloud->begin(), translated_cloud->end(),
                                      z_comp)->z;

        double xCoefficient = (img_size.width - 1) / 2.5;
        double yCoefficient = (img_size.height - 1) / 1.35;
        double zCoefficient = UCHAR_MAX / zMax;

        for (auto& point: *translated_cloud)
        {
            int yInd = (int) std::round(point.y * yCoefficient);
            int xInd = (int) std::round(point.x * xCoefficient);
            int zInd = (int) std::round(point.z * zCoefficient);

            auto value = CloudHandler::clamp<int>(UCHAR_MAX - zInd, 0, UCHAR_MAX);

            if (image.at<uchar>(yInd, xInd) < value)
            {
                image.at<uchar>(yInd, xInd) = value;
            }
        }


        cv::flip(image, image, 0);
        cv::imwrite(filename, image);
    }

    template<class PointT>
    static void ExportImageRGB(const typename PointCloud<PointT>::Ptr& translated_cloud,
                               std::string& filename,
                               const cv::Size& img_size,
                               int delta)
    {
        cv::Mat image = cv::Mat::zeros(img_size, CV_8UC3);

        auto z_comp = [](const PointT& p1, const PointT& p2) { return p1.z < p2.z; };

        float zMax = std::max_element(translated_cloud->begin(), translated_cloud->end(),
                                      z_comp)->z;

        double xCoefficient = (img_size.width - 1) / 2.5;
        double yCoefficient = (img_size.height - 1) / 1.35;
        double zCoefficient = UCHAR_MAX / zMax;

        for (auto& point: *translated_cloud)
        {
            int yInd = (int) std::round(point.y * yCoefficient);
            int xInd = (int) std::round(point.x * xCoefficient);
            int zInd = (int) std::round(point.z * zCoefficient);

            auto value = CloudHandler::clamp<int>(UCHAR_MAX - zInd, 0, UCHAR_MAX);

            if (image.at<uchar>(yInd, xInd) < value)
            {
                image.at<cv::Vec3b>(yInd, xInd) = {point.b, point.g, point.r};
            }
        }


        cv::flip(image, image, 0);

        //image = RemoveHolesWithReplace(RemoveHolesWithReplace(image, delta), delta);
        //image = RemoveHolesWithMeans(image, delta);
//        for (int i = 0; i < delta; ++i)
//            image = RemoveHolesWithReplace(image, 3);
//        for (int i = 0; i < delta; ++i)
//            image = RemoveHolesWithReplaceExpanding(image);
        cv::imwrite(filename, image);
    }

    template<class PointT>
    static void ExportPNGExperimental(const typename PointCloud<PointT>::Ptr& translated_cloud)
    {
        auto projection = ProjectToPlane(translated_cloud, {0, 0, 0}, {1, 0, 0}, {0, 1, 0});

        Visualize<PointXYZRGB>(projection);
    }

    template<class PointT>
    static void RotateX(const typename PointCloud<PointT>::Ptr& input,
                        float angle,
                        const typename PointCloud<PointT>::Ptr& output)
    {
        float theta = DEG2RAD(angle);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));

        pcl::transformPointCloud(*input, *output, transform);
    }

    template<class PointT>
    static void RotateY(const typename PointCloud<PointT>::Ptr& input,
                        float angle,
                        const typename PointCloud<PointT>::Ptr& output)
    {
        float theta = DEG2RAD(angle);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));

        pcl::transformPointCloud(*input, *output, transform);
    }

    template<class PointT>
    static void RotateZ(const typename PointCloud<PointT>::Ptr& input,
                        float angle,
                        const typename PointCloud<PointT>::Ptr& output)
    {
        float theta = DEG2RAD(angle);
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
        transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

        pcl::transformPointCloud(*input, *output, transform);
    }

    template<class PointT>
    static void Scale(const typename PointCloud<PointT>::Ptr& input,
                      const Vector3f& scaling,
                      const typename PointCloud<PointT>::Ptr& output)
    {
        Eigen::Affine3f transform = Eigen::Affine3f::Identity();

        transform.scale(scaling);

        transformPointCloud(*input, *output, transform);
    }

    template<class PointT>
    static void FlipX(const typename PointCloud<PointT>::Ptr& input,
                      const typename PointCloud<PointT>::Ptr& output)
    {
        Affine3f transform = Affine3f::Identity();

        Vector3f flip(-1, 1, 1);

        transform.scale(flip);

        transformPointCloud(*input, *output, transform);
    }

    template<class PointT>
    static void Augmentation(const typename PointCloud<PointT>::Ptr& cloud,
                             std::string& base_name,
                             bool flip)
    {
        typename PointCloud<PointT>::Ptr parallelepiped(new PointCloud<PointT>);
        PointCloud<PointT> cloud_copy;

        if (flip)
        {
            FlipX(cloud, cloud);
        }

        RotateX(cloud, -2, cloud);
        RotateY(cloud, -2, cloud);

        copyPointCloud(*cloud, cloud_copy);

        int id = 0;

        for (float i = 0.9; i <= 1.11; i += 0.02)
        {
            std::cout << i << std::endl;
            Scale(cloud, Eigen::Vector3f(i, i, i), cloud);

            for (int j = -2; j <= 2; ++j)
            {
                for (int k = -2; k <= 2; ++k)
                {
                    CreateParallelepiped(cloud, parallelepiped);
                    TranslateToBase(cloud, parallelepiped, cloud);
                    std::string filename = base_name + '.' + std::to_string(id) + ".bmp";
                    ExportImageDepth(cloud, filename);
                    parallelepiped = std::make_shared<PointCloud<PointXYZRGB>>();
                    RotateY(cloud, 1, cloud);
                    //Visualize(cloud);
                    ++id;
                }

                RotateX(cloud, 1, cloud);
                RotateY(cloud, -4, cloud);
            }

            copyPointCloud(cloud_copy, *cloud);
        }
    }

    static void Visualize(const PointCloud<PointXYZ>::Ptr& cloud,
                          Eigen::Vector4f& mean,
                          Eigen::Matrix3f& vectors);

    template<class PointT>
    static void Visualize(const typename PointCloud<PointT>::Ptr& cloud)
    {
        pcl::visualization::PCLVisualizer viewer;

        viewer.addPointCloud(cloud, "cloud");
        //viewer.addCoordinateSystem();
        viewer.setBackgroundColor(0, 0, 0);
        viewer.initCameraParameters();
        viewer.setCameraPosition(-0.2, -0.55, -1.42, 0, -0.55, 0, 0, 0, 0);
        viewer.setShowFPS(false);
        viewer.saveScreenshot("test.png");

        while (!viewer.wasStopped())
        {
            viewer.spinOnce(100);
        }
    }

    static cv::Mat RemoveHolesWithMeans(const cv::Mat& img, int delta);

    static std::list<cv::Vec3b> getPixelsInRadius(const cv::Mat& img, const cv::Point2i& point, int radius,
                                                  std::function<bool(const cv::Vec3b&)> pred);

    static PointCloud<PointXYZRGB>::Ptr
    ProjectToPlane(PointCloud<PointXYZRGB>::Ptr cloud, const Vector3f& origin, const Vector3f& axis_x,
                   const Vector3f& axis_y)
    {
        PointCloud<PointXYZRGB>::Ptr aux_cloud(new PointCloud<PointXYZRGB>);
        copyPointCloud(*cloud, *aux_cloud);

        auto normal = axis_x.cross(axis_y);
        Eigen::Hyperplane<float, 3> plane(normal, origin);

        for (auto & itPoint : *aux_cloud)
        {
            // project point to plane
            auto proj = plane.projection(itPoint.getVector3fMap());
            itPoint.getVector3fMap() = proj;
        }
        return aux_cloud;
    }

    static cv::Mat RemoveHolesWithReplace(const cv::Mat& img, int radius);

    static cv::Mat RemoveHolesWithReplaceExpanding(const cv::Mat& img);
};
