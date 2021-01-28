#include "CloudHandler.h"

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

void CloudHandler::CreateParallelepiped(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output, PointXYZ uncoloredPoint) {
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
    
    pcl::PassThrough<PointXYZ> psfilter;
    psfilter.setInputCloud(input);
    psfilter.setFilterLimits(yMin, yMin + 0.115);
    psfilter.setNegative(true);
    psfilter.setFilterFieldName("y");
    psfilter.filter(*input);
    
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

void CloudHandler::ExportImage(pcl::PointCloud<PointXYZ>::Ptr translated_cloud, bool flip, std::string filename)
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
    
    std::cout << image.cols << " " << image.rows << std::endl;
    
    for (auto point = translated_cloud->begin(); point != translated_cloud->end(); point++)
    {
        int yInd = std::round(point->y * yCoefficient);
        int xInd = std::round(point->x * xCoefficient);
        int zInd = std::round(point->z * zCoefficient);
        
        //std::cout << yInd << " " << xInd << std::endl;
        
        image.at<uchar>(yInd, xInd) = clamp(UCHAR_MAX - zInd, 0, UCHAR_MAX);
    }
    
    
    cv::flip(image, image, 0);
    if (flip)
        cv::flip(image, image, 1);
    
    cv::imwrite(filename, image);
}



