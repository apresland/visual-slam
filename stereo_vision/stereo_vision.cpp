#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "stereo_vision.h"

StereoMatcher::StereoMatcher()
{
    left_matcher = cv::StereoSGBM::create(0, max_disp, wsize);
    left_matcher->setP1(24*wsize*wsize);
    left_matcher->setP2(96*wsize*wsize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    right_matcher = cv::ximgproc::createRightMatcher(left_matcher);
    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
}

StereoMatcher::~StereoMatcher()
{
}

cv::Mat StereoMatcher::disparity(cv::Mat &left, cv::Mat &right)
{
    left_matcher->compute(left, right, left_disparity);
    right_matcher->compute(right, left, right_disparity);
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
    wls_filter->filter(left_disparity,left,filtered_disp,right_disparity);
    filtered_disp.convertTo(disparity_map, CV_32F, 1.0 / 16.0f);

    return disparity_map;
}


PointCloudGenerator::PointCloudGenerator(StereoMatcher &matcher, cv::Mat &Q)
        : stereoMatcher_(matcher)
        , Q_(Q)
{
}

PointCloudGenerator::~PointCloudGenerator()
{
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudGenerator::get(cv::Mat& left, cv::Mat& right)
{
    cv::Mat disparity = stereoMatcher_.disparity(left, right);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    pointcloud->width = static_cast<uint32_t>(disparity.cols);
    pointcloud->height = static_cast<uint32_t>(disparity.rows);
    pointcloud->is_dense = true;

    cv::Mat xyz;
    reprojectImageTo3D(disparity, xyz, Q_, true);

    pcl::PointXYZRGB point;
    for (int i = 0; i < disparity.rows; ++i)
    {
        uchar* rgb_ptr = left.ptr<uchar>(i);
        for (int j = 0; j < disparity.cols; ++j)
        {
            cv::Point3f p = xyz.at<cv::Point3f>(i, j);

            point.z = p.z;
            point.x = p.x;
            point.y = p.y;
            point.b = rgb_ptr[ j];
            point.g = rgb_ptr[ j];
            point.r = rgb_ptr[ j];
            pointcloud->points.push_back(point);
        }
    }

    // voxel grid filter
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (pointcloud);
    sor.setLeafSize (0.05, 0.05, 0.05);
    sor.filter (*cloud_filtered);

    return  cloud_filtered;
}