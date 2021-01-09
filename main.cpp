#include <iostream>
#include "CloudHandler.h"


bool loadCloud(const std::string& filename, pcl::PCLPointCloud2& cloud)
{
	TicToc tt;
	print_highlight("Loading "); print_value("%s ", filename.c_str());

	tt.tic();
	if (loadPLYFile(filename, cloud) < 0)
		return (false);
	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", cloud.width * cloud.height); print_info(" points]\n");
	print_info("Available dimensions: "); print_value("%s\n", pcl::getFieldsList(cloud).c_str());

	return (true);
}

void saveCloud(const std::string& filename, const pcl::PCLPointCloud2& cloud, bool binary, bool use_camera)
{
	TicToc tt;
	tt.tic();

	print_highlight("Saving "); print_value("%s ", filename.c_str());

	pcl::PLYWriter writer;
	writer.write(filename, cloud, Eigen::Vector4f::Zero(), Eigen::Quaternionf::Identity(), binary, use_camera);

	print_info("[done, "); print_value("%g", tt.toc()); print_info(" ms : "); print_value("%d", cloud.width * cloud.height); print_info(" points]\n");
}



int main (int argc, char** argv)
{
	// Load the first file
	pcl::PCLPointCloud2::Ptr loaded_cloud(new pcl::PCLPointCloud2());
	if (argc < 4 || !loadCloud(argv[1], *loaded_cloud))
        return (-1);

	CloudHandler cloud_handler;
	pcl::PCLPointCloud2::Ptr voxel_filtered_cloud(new pcl::PCLPointCloud2());
	cloud_handler.VoxelFilterCloud(loaded_cloud, voxel_filtered_cloud);

	pcl::PCLPointCloud2::Ptr upper(new pcl::PCLPointCloud2());
	pcl::PCLPointCloud2::Ptr lower(new pcl::PCLPointCloud2());
	cloud_handler.CutCloud(voxel_filtered_cloud, upper, lower);
	//saveCloud("upper.ply", *upper, false, false);
	//saveCloud("lower.ply", *lower, false, false);

	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*lower, *cloud);
	// Find grownd plane
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::copyPointCloud(*cloud, *plane);

	cloud_handler.FindPlane(plane, inliers);
	pcl::PCLPointCloud2::Ptr plane_cloud(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*plane, *plane_cloud);
	//saveCloud("green_plane.ply", *plane_cloud, false, false);


	// save hooves
	pcl::PointCloud<pcl::PointXYZ>::Ptr hooves(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::copyPointCloud(*cloud, *hooves);
	pcl::ExtractIndices<pcl::PointXYZ> extract;
	extract.setInputCloud(hooves);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(*hooves);

	pcl::PCLPointCloud2::Ptr hooves_cloud(new pcl::PCLPointCloud2());
	pcl::toPCLPointCloud2(*hooves, *hooves_cloud);
	//saveCloud("hooves.ply", *hooves_cloud, false, false);

	// add hooves to cow cloud
	pcl::PointCloud<pcl::PointXYZ>::Ptr cow_cloud(new pcl::PointCloud<pcl::PointXYZ>);
	pcl::fromPCLPointCloud2(*upper, *cow_cloud);

	for (auto p : hooves->points)
		cow_cloud->insert(cow_cloud->end(), 1, p);
	pcl::toPCLPointCloud2(*cow_cloud, *upper);


	pcl::PCLPointCloud2::Ptr cow(new pcl::PCLPointCloud2());
	cloud_handler.RorFilterCloud(upper, cow);
	//saveCloud("cow.ply", *cow, false, false);

	// find pca
	pcl::PCA<pcl::PointXYZ> pca;
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

    
    //get cow
    pcl::PointCloud<PointXYZ>::Ptr cow_only(new pcl::PointCloud<PointXYZ>);
    pcl::PointCloud<PointXYZ>::Ptr cows_parallelepiped(new pcl::PointCloud<PointXYZ>);
    pcl::PassThrough<PointXYZ> PTfilter;
    PTfilter.setInputCloud(cow_cloud);
    PTfilter.setFilterFieldName("z");
    PTfilter.setFilterLimits(1.75, 3);
    PTfilter.filter(*cow_only);
    PointXYZ uncoloredPoint;
    cloud_handler.CreateParallelepiped(cow_only, cows_parallelepiped, uncoloredPoint);
    
    // translate cow
    pcl::PointCloud<PointXYZ>::Ptr translated_cow(new pcl::PointCloud<PointXYZ>);
    cloud_handler.TranslateToBase(cow_only, cows_parallelepiped, translated_cow);
    PTfilter.setInputCloud(translated_cow);
    PTfilter.setFilterLimits(0, 0.115);
    PTfilter.setFilterFieldName("y");
    PTfilter.setNegative(true);
    PTfilter.filter(*translated_cow);
    
    // очень сомнительная вещь
    /*pcl::RadiusOutlierRemoval<PointXYZ> ROR;
    ROR.setInputCloud(translated_cow);
    ROR.setRadiusSearch(0.1);
    ROR.setMinNeighborsInRadius(80);
    ROR.filter(*translated_cow);
    ROR.filter(*translated_cow);*/
    
    
    // save PNG
    cloud_handler.ExportImage(translated_cow, std::atoi(argv[2]), argv[3]);
    
    
	// visualize
	pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Visualization"));
    viewer->addPointCloud(translated_cow, "translated cow");
    viewer->addCoordinateSystem();
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();

	//viewer->addPointCloudNormals<pcl::PointXYZ, pcl::Normal>(cow_cloud, normals);

	PointXYZ center = PointXYZ(mean[0], mean[1], mean[2]);
	PointXYZ p1(center); p1.x += vects(0, 0); p1.y += vects(1, 0); p1.z += vects(2, 0);
	PointXYZ p2(center); p2.x += vects(0, 1); p2.y += vects(1, 1); p2.z += vects(2, 1);
	PointXYZ p3(center); p3.x += vects(0, 2); p3.y += vects(1, 2); p3.z += vects(2, 2);

	pcl::ModelCoefficients::Ptr plane_xy(new pcl::ModelCoefficients);
	plane_xy->values = { vects(0, 2), vects(1, 2), vects(2, 2), std::abs(center.z) };
	pcl::ModelCoefficients::Ptr plane_xz(new pcl::ModelCoefficients);
	plane_xz->values = { vects(0, 1), vects(1, 1), vects(2, 1), std::abs(center.y) };
	pcl::ModelCoefficients::Ptr plane_yz(new pcl::ModelCoefficients);
	plane_yz->values = { vects(0, 0), vects(1, 0), vects(2, 0), std::abs(center.x) };

	viewer->addLine(center, p1, 1, 0, 0, "xline");
	viewer->addLine(center, p2, 0, 1, 0, "yline");
	viewer->addLine(center, p3, 0, 0, 1, "zline");

	std::map<std::string, pcl::ModelCoefficients::Ptr> planes;
	planes["plane_xy"] = plane_xy;
	planes["plane_xz"] = plane_xz;
	planes["plane_yz"] = plane_yz;

	int color = 0;
	for (auto plane : planes)
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
		viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_REPRESENTATION, pcl::visualization::PCL_VISUALIZER_REPRESENTATION_SURFACE, plane.first, 0);
	}

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		//boost::this_thread::sleep_for(boost::posix_time::microseconds(100000));
	}

	return 0;
}
