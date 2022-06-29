#include "CloudHandler.hpp"
#include <memory>

void CloudHandler::VoxelFilterCloud(const pcl::PCLPointCloud2::Ptr& input,
                                    const pcl::PCLPointCloud2::Ptr& output)
{
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud(input);
    sor.setLeafSize(0.01f, 0.01f, 0.01f);
    sor.filter(*output);
}

void CloudHandler::RorFilterCloud(const pcl::PCLPointCloud2::Ptr& input,
                                  const pcl::PCLPointCloud2::Ptr& output)
{
    pcl::RadiusOutlierRemoval<pcl::PCLPointCloud2> ror;
    ror.setRadiusSearch(0.05);
    ror.setMinNeighborsInRadius(20);
    ror.setInputCloud(input);
    ror.filter(*output);
}


void CloudHandler::CutCloud(const pcl::PCLPointCloud2::Ptr& input,
                            const pcl::PCLPointCloud2::Ptr& upper,
                            const pcl::PCLPointCloud2::Ptr& lower)
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

void CloudHandler::Visualize(const PointCloud<PointXYZ>::Ptr& cloud, Eigen::Vector4f& mean, Eigen::Matrix3f& vectors)
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
    for (const auto& plane: planes)
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

std::list<cv::Vec3b> CloudHandler::getPixelsInRadius(const cv::Mat& img, const cv::Point2i& point, int radius,
                                                     std::function<bool(const cv::Vec3b&)> pred)
{
    const cv::Size img_size = {img.cols, img.rows};
    std::list<cv::Vec3b> result;

    for (int iteration = 1; iteration <= radius; ++iteration)
    {
        cv::Vec2i upperLeftPoint = {point.x - iteration, point.y - iteration};
        cv::Vec2i bottomRightPoint = {point.x + iteration, point.y + iteration};

        for (int p_x = upperLeftPoint[0]; p_x < bottomRightPoint[0]; ++p_x)
        {
            if (p_x >= 0 && p_x < img_size.width)
            {

                if (upperLeftPoint[1] >= 0 && upperLeftPoint[1] < img_size.height)
                {
                    //std::cout << "Upper point: " << p_x << ", " << upperLeftPoint[1] << '\n';
                    auto& p = img.at<cv::Vec3b>(upperLeftPoint[1], p_x);
                    if (pred(p))
                        result.push_back(p);
                }
                if (bottomRightPoint[1] >= 0 && bottomRightPoint[1] < img_size.height)
                {
                    //std::cout << "Bottom point: " << p_x << ", " << bottomRightPoint[1] << '\n';
                    auto& p = img.at<cv::Vec3b>(bottomRightPoint[1], p_x);
                    if (pred(p))
                        result.push_back(p);
                }
            }
        }

        for (int p_y = upperLeftPoint[1]; p_y < bottomRightPoint[1]; ++p_y)
        {
            if (p_y >= 0 && p_y < img_size.height)
            {
                if (upperLeftPoint[0] >= 0 && upperLeftPoint[0] < img_size.width)
                {
                    //std::cout << "Upper point: " << upperLeftPoint[0] << ", " << p_y << '\n';
                    auto& p = img.at<cv::Vec3b>(p_y, upperLeftPoint[0]);
                    if (pred(p))
                        result.push_back(p);
                }
                if (bottomRightPoint[0] >= 0 && bottomRightPoint[0] < img_size.width)
                {
                    //std::cout << "Bottom point: " << bottomRightPoint[0] << ", " << p_y << '\n';
                    auto& p = img.at<cv::Vec3b>(p_y, bottomRightPoint[0]);
                    if (pred(p))
                        result.push_back(p);
                }
            }
        }
    }

    return std::move(result);
}

cv::Mat CloudHandler::RemoveHolesWithMeans(const cv::Mat& img, int delta)
{
    cv::Mat new_img;
    img.copyTo(new_img);

    for (int y = 0; y < img.rows; ++y)
    {
        for (int x = 0; x < img.cols; x += 1)
        {
            auto pixels = getPixelsInRadius(img, {x, y}, delta,
                                            [](const cv::Vec3b& p)
                                            {
                                                return p != cv::Vec3b{0, 0, 0};
                                            });
            if (pixels.size() >= 5)
            {
                cv::Vec3i colors = {0, 0, 0};
                for (auto& pix: pixels)
                    colors += pix;
                for (int i = 0; i < 3; ++i)
                    colors[i] /= (int)pixels.size();
                new_img.at<cv::Vec3b>(y, x) = colors;
            }
        }
    }
    //new_img.at<cv::Vec3b>(0, 0) = {0, 0, 255};

    return new_img;
}

cv::Mat CloudHandler::RemoveHolesWithReplace(const cv::Mat& img, int radius)
{
    static const cv::Vec3b blackPoint = {0, 0, 0};
    cv::Mat new_img;
    img.copyTo(new_img);

    for (int y = 0 + radius; y < img.rows - radius; ++y)
    {
        for (int x = 0 + radius; x < img.cols - radius; ++x)
        {
            if (img.at<cv::Vec3b>(y, x) != blackPoint)
                continue;

            auto pixels = getPixelsInRadius(img, {x, y}, radius,
                                            [](const cv::Vec3b& p)
                                            {
                                                return p != cv::Vec3b{0, 0, 0};
                                            });

            if (pixels.empty())
                continue;

            new_img.at<cv::Vec3b>(y, x) = pixels.front();
            x += radius * 2;
        }
    }

    return std::move(new_img);
}

cv::Mat CloudHandler::RemoveHolesWithReplaceExpanding(const cv::Mat& img)
{
    const int radius = 7;
    const cv::Vec3b blackPoint = {0, 0, 0};
    cv::Mat new_img;
    img.copyTo(new_img);

    for (int y = 0 + radius; y < img.rows - radius; ++y)
    {
        for (int x = 0 + radius; x < img.cols - radius; ++x)
        {
            if (img.at<cv::Vec3b>(y, x) != blackPoint)
                continue;
            std::list<cv::Vec3b> pixels;

            for (auto inc_radius = 3; pixels.empty() && inc_radius <= 7; inc_radius += 2)
            {
                pixels = getPixelsInRadius(img, {x, y}, inc_radius,
                                           [](const cv::Vec3b& p)
                                           {
                                               return p != cv::Vec3b{0, 0, 0};
                                           });
            }

            if (pixels.empty())
                continue;

            new_img.at<cv::Vec3b>(y, x) = pixels.front();
            x += radius * 2;
        }
    }

    return std::move(new_img);
}

