#include <iostream>
#include "CloudHandler.h"


bool loadCloud(const std::string &filename, pcl::PCLPointCloud2 &cloud)
{
    TicToc tt;
    print_highlight("Loading ");
    print_value("%s ", filename.c_str());

    tt.tic();
    if (loadPLYFile(filename, cloud) < 0)
        return (false);
    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_value("%d", cloud.width * cloud.height);
    print_info(" points]\n");
    print_info("Available dimensions: ");
    print_value("%s\n", pcl::getFieldsList(cloud).c_str());

    return (true);
}

void saveCloud(const std::string &filename, const pcl::PCLPointCloud2 &cloud, bool binary, bool use_camera)
{
    TicToc tt;
    tt.tic();

    print_highlight("Saving ");
    print_value("%s ", filename.c_str());

    pcl::PLYWriter writer;
    writer.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary, use_camera);

    print_info("[done, ");
    print_value("%g", tt.toc());
    print_info(" ms : ");
    print_value("%d", cloud.width * cloud.height);
    print_info(" points]\n");
}


int main(int argc, char **argv)
{
    // Load the first file
    pcl::PCLPointCloud2::Ptr loaded_cloud(new pcl::PCLPointCloud2());
    if (argc < 4 || !loadCloud(argv[1], *loaded_cloud))
        return (-1);

    pcl::PassThrough<PCLPointCloud2> passfilter;
    passfilter.setInputCloud(loaded_cloud);
    passfilter.setFilterLimits(1.60, 3);
    passfilter.setFilterFieldName("z");
    passfilter.filter(*loaded_cloud);

    pcl::PCLPointCloud2::Ptr voxel_filtered_cloud(new pcl::PCLPointCloud2());
    CloudHandler::VoxelFilterCloud(loaded_cloud, voxel_filtered_cloud);

    pcl::PCLPointCloud2::Ptr upper(new pcl::PCLPointCloud2());
    pcl::PCLPointCloud2::Ptr lower(new pcl::PCLPointCloud2());
    CloudHandler::CutCloud(voxel_filtered_cloud, upper, lower);
    //saveCloud("upper.ply", *upper, false, false);
    //saveCloud("lower.ply", *lower, false, false);


    PointCloud<PointXYZRGB>::Ptr cloud(new PointCloud<PointXYZRGB>);
    fromPCLPointCloud2(*lower, *cloud);
    // Find grownd plane
    PointIndices::Ptr inliers(new pcl::PointIndices);
    PointCloud<PointXYZRGB>::Ptr plane(new pcl::PointCloud<PointXYZRGB>);
    copyPointCloud(*cloud, *plane);

    auto model_coef = CloudHandler::FindPlane<PointXYZRGB>(plane, inliers);
    pcl::PCLPointCloud2::Ptr plane_cloud(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*plane, *plane_cloud);
    //saveCloud("green_plane.ply", *plane_cloud, false, false);

    auto angle = acos(abs(model_coef->values[1]) / std::sqrt(
            std::pow(model_coef->values[0], 2) + std::pow(model_coef->values[1], 2) +
            std::pow(model_coef->values[2], 2)));

    std::cout << "Angle: " << angle << std::endl;

    Eigen::Matrix4f rot_y;

    rot_y << cos(angle), 0, sin(angle), 0,
            0, 1, 0, 0,
            -sin(angle), 0, cos(angle), 0,
            0, 0, 0, 1;



    // save hooves
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr hooves(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::copyPointCloud(*cloud, *hooves);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    extract.setInputCloud(hooves);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*hooves);

    pcl::PCLPointCloud2::Ptr hooves_cloud(new pcl::PCLPointCloud2());
    pcl::toPCLPointCloud2(*hooves, *hooves_cloud);
    //saveCloud("hooves.ply", *hooves_cloud, false, false);

    // add hooves to cow cloud
    shared_ptr<PointCloud<PointXYZRGB>> cow_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(*upper, *cow_cloud);

    for (auto p: hooves->points)
        cow_cloud->insert(cow_cloud->end(), 1, p);
    pcl::toPCLPointCloud2(*cow_cloud, *upper);


    pcl::PCLPointCloud2::Ptr cow(new pcl::PCLPointCloud2());
    CloudHandler::RorFilterCloud(upper, cow);
    //saveCloud("cow.ply", *cow, false, false);

    // find pca
    pcl::PCA<pcl::PointXYZRGB> pca;
    pcl::fromPCLPointCloud2(*cow, *cow_cloud);
    pca.setInputCloud(cow_cloud);
    auto mean = pca.getMean();
    std::cout << "mean: " << std::endl << mean << std::endl;
    //auto coefs = pca.getCoefficients();
    //std::cout << "coefs: " << std::endl << coefs << std::endl;
    auto vals = pca.getEigenValues();
    std::cout << "vals: " << std::endl << vals << std::endl;
    auto vects = pca.getEigenVectors();
    std::cout << "vects: " << std::endl << vects << std::endl;

    Eigen::Matrix4f rotation_x, rotation_y, rotation_z;

    auto xAngle = acos(abs(vects(0, 0)) /
                       std::sqrt(std::pow(vects(0, 0), 2) + std::pow(vects(0, 1), 2) + std::pow(vects(0, 2), 2)));

    auto yAngle = acos(abs(vects(1, 1)) /
                       std::sqrt(std::pow(vects(1, 0), 2) + std::pow(vects(1, 1), 2) + std::pow(vects(1, 2), 2)));

    auto zAngle = acos(abs(vects(2, 2)) /
                       std::sqrt(std::pow(vects(2, 0), 2) + std::pow(vects(2, 1), 2) + std::pow(vects(2, 2), 2)));

    std::cout << "x,y,z = " << xAngle << " " << yAngle << " " << zAngle << std::endl;

    rotation_x << 1, 0, 0, 0,
            0, cos(xAngle), -sin(xAngle), 0,
            0, sin(xAngle), cos(xAngle), 0,
            0, 0, 0, 1;

    rotation_y << cos(yAngle), 0, sin(yAngle), 0,
            0, 1, 0, 0,
            -sin(yAngle), 0, cos(yAngle), 0,
            0, 0, 0, 1;

    rotation_z << cos(zAngle), -sin(zAngle), 0, 0,
            sin(zAngle), cos(zAngle), 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1;


    // get cow
    pcl::PointCloud<PointXYZRGB>::Ptr cows_parallelepiped(new pcl::PointCloud<PointXYZRGB>);
    pcl::transformPointCloud(*cow_cloud, *cow_cloud, rot_y);
    pcl::transformPointCloud(*cow_cloud, *cow_cloud, rotation_x);
    //pcl::transformPointCloud(*cow_cloud, *cow_cloud, rotation_z);

    CloudHandler::FlipX<PointXYZRGB>(cow_cloud, cow_cloud);
    CloudHandler::CreateParallelepiped<PointXYZRGB>(cow_cloud, cows_parallelepiped);

    // translate cow
    PointCloud<PointXYZRGB>::Ptr translated_cow(new pcl::PointCloud<PointXYZRGB>);
    CloudHandler::TranslateToBase<PointXYZRGB>(cow_cloud, cows_parallelepiped, translated_cow);

    pcl::PassThrough<PointXYZRGB> PRfilter;
    PRfilter.setInputCloud(translated_cow);
    PRfilter.setFilterFieldName("y");
    PRfilter.setFilterLimits(0.115, 100);
    PRfilter.filter(*translated_cow);

    //cloud_handler.RorFilterCloud(translated_cow, translated_cow);


    //xAngle = atan2(Vector3f(1, 0, 0), std::sqrt(std::pow(vects(0,0), 2) + std::pow(vects(0,1), 2) + std::pow(vects(0, 2), 2)));

    // rotation
    //pcl::transformPointCloud(*translated_cow, *translated_cow, rotation_x);
    //pcl::transformPointCloud(*translated_cow, *translated_cow, rotation_y);
    //pcl::pcl::transformPointCloud(*translated_cow, *translated_cow, rotation_x);

    /*auto xMax = translated_cow->begin()->x;
    auto yMax = translated_cow->begin()->y;
    auto zMax = translated_cow->begin()->z;
    
    for (auto& point : *translated_cow)
    {
        xMax = std::max(point.x, xMax);
        yMax = std::max(point.y, yMax);
        zMax = std::max(point.z, zMax);
    }
    
    std::ofstream output("xyz.txt", std::ios::app);
    output << xMax << " " << yMax << " " << zMax << std::endl;
    output.close();*/

    // очень сомнительная вещь
    /*pcl::RadiusOutlierRemoval<PointXYZ> ROR;
    ROR.setInputCloud(translated_cow);
    ROR.setRadiusSearch(0.1);
    ROR.setMinNeighborsInRadius(80);
    ROR.filter(*translated_cow);
    ROR.filter(*translated_cow);
     
     найти XYZ макс среди всех облаков
     
     
     */


    // save image
    //cloud_handler.Augmentation(translated_cow, argv[3], std::atoi(argv[2]));
    //cloud_handler.FlipX(translated_cow, translated_cow);
    std::string filename = argv[3];
    CloudHandler::ExportImageDepth<PointXYZRGB>(translated_cow, filename);
    //cloud_handler.Visualize(translated_cow, mean, vects);

    return 0;
}
