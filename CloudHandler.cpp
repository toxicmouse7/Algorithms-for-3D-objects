#include "CloudHandler.h"

///Returns a value between min and max
///@param value Value
///@param min Minimal
///@param max Maximal
template <class T>
T CloudHandler::clamp(T value, T min, T max)
{
    if (value < min)
        return min;
    if (value > max)
        return max;
    return value;
}

void CloudHandler::VoxelFilterCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr output) {
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud(input);
	sor.setLeafSize(0.01f, 0.01f, 0.01f);
	sor.filter(*output);
}

void CloudHandler::RorFilterCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr output) {
	pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
	ror.setRadiusSearch(0.05);
	ror.setMinNeighborsInRadius(20);
	ror.setInputCloud(input);
	ror.filter(*output);
}

void CloudHandler::RorFilterCloud(pcl::PointCloud<PointXYZ>::Ptr input, pcl::PointCloud<PointXYZ>::Ptr output)
{
    pcl::RadiusOutlierRemoval<PointXYZ> ror;
    ror.setRadiusSearch(0.025);
    ror.setMinNeighborsInRadius(10);
    ror.setInputCloud(input);
    ror.filter(*output);
}

pcl::ModelCoefficients::Ptr CloudHandler::FindPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, pcl::PointIndices::Ptr inliers) {
	// Find grownd plane
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
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

	if (inliers->indices.size() == 0)
	{
		PCL_ERROR("Could not estimate a planar model for the given dataset.");
	}

	std::cerr << "Model coefficients: " << coefficients->values[0] << " "
		<< coefficients->values[1] << " "
		<< coefficients->values[2] << " "
		<< coefficients->values[3] << std::endl;


	std::cerr << "Model inliers: " << inliers->indices.size() << std::endl;


	for (std::size_t i = 0; i < inliers->indices.size(); ++i) {
		cloud->points[inliers->indices[i]].r = 0;
		cloud->points[inliers->indices[i]].g = 255;
		cloud->points[inliers->indices[i]].b = 0;
		cloud->points[inliers->indices[i]].a = 255;
	}

	for (size_t i = 0; i < cloud->points.size(); i++)
		if (cloud->points[i].g != 255) {
			cloud->erase(cloud->begin() + i, cloud->begin() + i + 1);
			i--;
		}
    
    return coefficients;
}


void CloudHandler::CutCloud(pcl::PCLPointCloud2::Ptr input, pcl::PCLPointCloud2::Ptr upper, pcl::PCLPointCloud2::Ptr lower) {
	pcl::PassThrough<pcl::PCLPointCloud2> pass;
	pass.setInputCloud(input);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(-100, -0.5);
	pass.filter(*lower);

	//pass.setFilterLimitsNegative(true);
    pass.setNegative(true);
	pass.filter(*upper);
}


void CloudHandler::CreateParallelepiped(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output, PointXYZRGB coloredPoint) {
	float xMin = input->begin()->x;
	float xMax = xMin;
	float yMin = input->begin()->y;
	float yMax = yMin;
	float zMin = input->begin()->z;
	float zMax = zMin;

	for (auto i = input->begin(); i != input->end(); ++i)
	{
		xMin = std::min(xMin, i->x);
		yMin = std::min(yMin, i->y);
		zMin = std::min(zMin, i->z);

		xMax = std::max(xMax, i->x);
		yMax = std::max(yMax, i->y);
		zMax = std::max(zMax, i->z);
	}

	// create parallelepiped around cloud
	for (float x = xMin; x < xMax; x += 0.01)
	{
		coloredPoint.x = x; coloredPoint.y = yMin; coloredPoint.z = zMin;
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
		coloredPoint.x = xMin; coloredPoint.y = y; coloredPoint.z = zMin;
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
		coloredPoint.x = xMin; coloredPoint.y = yMin; coloredPoint.z = z;
		output->push_back(coloredPoint);
		coloredPoint.x = xMax;
		output->push_back(coloredPoint);
		coloredPoint.y = yMax;
		output->push_back(coloredPoint);
		coloredPoint.x = xMin;
		output->push_back(coloredPoint);
	}
}

void CloudHandler::CreateParallelepiped(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output)
{
    PointXYZ uncoloredPoint;
    
    float xMin = input->begin()->x;
    float xMax = xMin;
    float yMin = input->begin()->y;
    float yMax = yMin;
    float zMin = input->begin()->z;
    float zMax = zMin;

    for (auto i = input->begin(); i != input->end(); ++i)
    {
        yMin = std::min(yMin, i->y);
    }
    
    for (auto i = input->begin(); i != input->end(); ++i)
    {
        xMin = std::min(xMin, i->x);
        yMin = std::min(yMin, i->y);
        zMin = std::min(zMin, i->z);

        xMax = std::max(xMax, i->x);
        yMax = std::max(yMax, i->y);
        zMax = std::max(zMax, i->z);
    }

    // create parallelepiped around cloud
    for (float x = xMin; x < xMax; x += 0.01)
    {
        uncoloredPoint.x = x; uncoloredPoint.y = yMin; uncoloredPoint.z = zMin;
        output->push_back(uncoloredPoint);
        uncoloredPoint.y = yMax;
        output->push_back(uncoloredPoint);
        uncoloredPoint.z = zMax;
        output->push_back(uncoloredPoint);
        uncoloredPoint.y = yMin;
        output->push_back(uncoloredPoint);
    }
    for (float y = yMin; y < yMax; y += 0.01)
    {
        uncoloredPoint.x = xMin; uncoloredPoint.y = y; uncoloredPoint.z = zMin;
        output->push_back(uncoloredPoint);
        uncoloredPoint.x = xMax;
        output->push_back(uncoloredPoint);
        uncoloredPoint.z = zMax;
        output->push_back(uncoloredPoint);
        uncoloredPoint.x = xMin;
        output->push_back(uncoloredPoint);
    }
    for (float z = zMin; z < zMax; z += 0.01)
    {
        uncoloredPoint.x = xMin; uncoloredPoint.y = yMin; uncoloredPoint.z = z;
        output->push_back(uncoloredPoint);
        uncoloredPoint.x = xMax;
        output->push_back(uncoloredPoint);
        uncoloredPoint.y = yMax;
        output->push_back(uncoloredPoint);
        uncoloredPoint.x = xMin;
        output->push_back(uncoloredPoint);
    }
}

void CloudHandler::TranslateToBase(pcl::PointCloud<PointXYZ>::Ptr input, pcl::PointCloud<PointXYZ>::Ptr parallelepiped, pcl::PointCloud<PointXYZ>::Ptr output)
{
    float xMin = parallelepiped->begin()->x;
    float yMin = parallelepiped->begin()->y;
    float zMin = parallelepiped->begin()->z;
    
    for (auto i = parallelepiped->begin(); i != parallelepiped->end(); ++i)
    {
        xMin = std::min(xMin, i->x);
        yMin = std::min(yMin, i->y);
        zMin = std::min(zMin, i->z);
    }
    
    Eigen::Matrix4f first_translation(4,4);
    first_translation << 1, 0, 0, -xMin,
                         0, 1, 0, -yMin,
                         0, 0, 1, -zMin,
                         0, 0, 0, 1;
    pcl::transformPointCloud(*input, *output, first_translation);
}

void CloudHandler::ProjectOnXOY(pcl::PointCloud<PointXYZ>::Ptr input, pcl::PointCloud<PointXYZ>::Ptr output)
{
    //might be used for auto-translating
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    coefficients->values.resize(4);
    coefficients->values[0] = coefficients->values[1] = 0;
    coefficients->values[2] = 1;
    coefficients->values[3] = 1;
    
    pcl::ProjectInliers<PointXYZ> proj;
    proj.setModelType(pcl::SACMODEL_PLANE);
    proj.setInputCloud(input);
    proj.setModelCoefficients(coefficients);
    proj.filter(*output);
    std::cout << output->points.size() << " - points in projection" << std::endl;
    std::cout << input->points.size() << " - points in original" << std::endl;
}

void CloudHandler::ExportImage(pcl::PointCloud<PointXYZ>::Ptr translated_cloud, std::string filename)
{
    cv::Size img_size(299, 150);
    cv::Mat image = cv::Mat::zeros(img_size, CV_8UC1);
    
    float xMax = translated_cloud->begin()->x;
    float yMax = translated_cloud->begin()->y;
    float zMax = translated_cloud->begin()->z;
    
    for (auto& point : *translated_cloud)
    {
        xMax = std::max(xMax, point.x);
        yMax = std::max(yMax, point.y);
        zMax = std::max(zMax, point.z);
    }
    
    float xCoefficient = (img_size.width - 1) / 2.5;
    float yCoefficient = (img_size.height - 1) / 1.35;
    float zCoefficient = UCHAR_MAX / zMax;
    
    for (auto point = translated_cloud->begin(); point != translated_cloud->end(); point++)
    {
        int yInd = std::round(point->y * yCoefficient);
        int xInd = std::round(point->x * xCoefficient);
        int zInd = std::round(point->z * zCoefficient);
        
        auto value = clamp(UCHAR_MAX - zInd, 0, UCHAR_MAX);
        
        if (image.at<uchar>(yInd, xInd) < value)
        {
            image.at<uchar>(yInd, xInd) = value;
        }
    }
    
    
    cv::flip(image, image, 0);
    cv::imwrite(filename, image);
}

void CloudHandler::RotateX(pcl::PointCloud<PointXYZ>::Ptr input, float angle, PointCloud<PointXYZ>::Ptr output)
{
    float theta = DEG2RAD(angle);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitX()));
    
    pcl::transformPointCloud(*input, *output, transform);
}

void CloudHandler::RotateY(pcl::PointCloud<PointXYZ>::Ptr input, float angle, PointCloud<PointXYZ>::Ptr output)
{
    float theta = DEG2RAD(angle);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY()));
    
    pcl::transformPointCloud(*input, *output, transform);
}

///Rotates *cloud* for *angle* degrees about Z-axis
void CloudHandler::RotateZ(pcl::PointCloud<PointXYZ>::Ptr input, float angle, PointCloud<PointXYZ>::Ptr output)
{
    float theta = DEG2RAD(angle);
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    transform.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));
    
    pcl::transformPointCloud(*input, *output, transform);
}

void CloudHandler::Augmentation(pcl::PointCloud<PointXYZ>::Ptr cloud, std::string base_name, bool flip)
{
    PointCloud<PointXYZ>::Ptr parallelepiped(new PointCloud<PointXYZ>);
    PointCloud<PointXYZ> cloud_copy;
    
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
                ExportImage(cloud, base_name + '.' + std::to_string(id) + ".bmp");
                parallelepiped.reset(new PointCloud<PointXYZ>);
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

void CloudHandler::Visualize(PointCloud<PointXYZ>::Ptr cloud, Eigen::Vector4f& mean, Eigen::Matrix3f& vects)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Visualization"));
    viewer->addPointCloud(cloud, "translated cow");
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
}

void CloudHandler::Visualize(PointCloud<PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer viewer;
    
    viewer.addPointCloud(cloud, "cloud");
    viewer.addCoordinateSystem();
    viewer.setBackgroundColor(0, 0, 0);
    viewer.initCameraParameters();
    
    while (!viewer.wasStopped())
    {
        viewer.spinOnce(100);
    }
}

void CloudHandler::Scale(PointCloud<PointXYZ>::Ptr input, Eigen::Vector3f scaling, PointCloud<PointXYZ>::Ptr output)
{
    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    
    transform.scale(scaling);
    
    transformPointCloud(*input, *output, transform);
}

void CloudHandler::FlipX(PointCloud<PointXYZ>::Ptr input, PointCloud<PointXYZ>::Ptr output)
{
    Affine3f transform = Affine3f::Identity();
    
    Vector3f flip(-1, 1, 1);
    
    transform.scale(flip);
    
    transformPointCloud(*input, *output, transform);
}


