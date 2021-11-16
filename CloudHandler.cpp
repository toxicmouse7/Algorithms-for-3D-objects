#include "CloudHandler.h"
#include <memory>

void CloudHandler::VoxelFilterCloud(const pcl::PCLPointCloud2::Ptr &input,
                                    const pcl::PCLPointCloud2::Ptr &output)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*output);
}

void CloudHandler::RorFilterCloud(const pcl::PCLPointCloud2::Ptr &input,
                                  const pcl::PCLPointCloud2::Ptr &output)
{
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
    ror.setRadiusSearch(0.05);
    ror.setMinNeighborsInRadius(20);
    ror.setInputCloud(input);
    ror.filter(*output);
}


void CloudHandler::CutCloud(const pcl::PCLPointCloud2::Ptr &input,
                            const pcl::PCLPointCloud2::Ptr &upper,
                            const pcl::PCLPointCloud2::Ptr &lower)
{
    pcl::PassThrough<pcl::PCLPointCloud2> pass;
    pass.setInputCloud(input);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(-100, -0.5);
    pass.filter(*lower);

    //pass.setFilterLimitsNegative(true);
    pass.setNegative(true);
    pass.filter(*upper);
}

void CloudHandler::Visualize(const PointCloud<PointXYZ>::Ptr &cloud, Eigen::Vector4f &mean, Eigen::Matrix3f &vectors)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Visualization"));
    viewer->addPointCloud(cloud, "translated cow");
    viewer->addCoordinateSystem();
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    //viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cow_cloud, normals);

    PointXYZ center = PointXYZ(mean[0], mean[1], mean[2]);
    PointXYZ p1(center);
    p1.x += vectors(0, 0);
    p1.y += vectors(1, 0);
    p1.z += vectors(2, 0);
    PointXYZ p2(center);
    p2.x += vectors(0, 1);
    p2.y += vectors(1, 1);
    p2.z += vectors(2, 1);
    PointXYZ p3(center);
    p3.x += vectors(0, 2);
    p3.y += vectors(1, 2);
    p3.z += vectors(2, 2);

    pcl::ModelCoefficients::Ptr plane_xy(new pcl::ModelCoefficients);
    plane_xy->values = {vectors(0, 2), vectors(1, 2), vectors(2, 2), std::abs(center.z)};
    pcl::ModelCoefficients::Ptr plane_xz(new pcl::ModelCoefficients);
    plane_xz->values = {vectors(0, 1), vectors(1, 1), vectors(2, 1), std::abs(center.y)};
    pcl::ModelCoefficients::Ptr plane_yz(new pcl::ModelCoefficients);
    plane_yz->values = {vectors(0, 0), vectors(1, 0), vectors(2, 0), std::abs(center.x)};

    viewer->addLine(center, p1, 1, 0, 0, "xline");
    viewer->addLine(center, p2, 0, 1, 0, "yline");
    viewer->addLine(center, p3, 0, 0, 1, "zline");

    std::map<std::string, pcl::ModelCoefficients::Ptr> planes;
    planes["plane_xy"] = plane_xy;
    planes["plane_xz"] = plane_xz;
    planes["plane_yz"] = plane_yz;

    int color = 0;
    for (const auto &plane: planes)
    {
        viewer->addPlane(*plane.second, plane.first, 0);
        switch (color)
        {
            case 0:
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 1, 0, 0, plane.first, 0);
                break;
            case 1:
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 0, 1, plane.first, 0);
                break;
            default:
                viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, 0, 1, 0, plane.first, 0);
                break;
        }
        color++;
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.5, plane.first, 0);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                                            pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, plane.first, 0);
    }

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
        //boost::this_thread::sleep_for(boost::posix_time::microseconds(100000));
    }
}