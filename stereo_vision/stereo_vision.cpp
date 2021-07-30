#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include "stereo_vision.h"

/**
 * Constructs a stereo matcher with default settings.
 */
StereoMatcher::StereoMatcher()
{
    left_matcher = cv::StereoSGBM::create(0, max_disp, wsize);
    left_matcher->setP1(24*wsize*wsize);
    left_matcher->setP2(96*wsize*wsize);
    left_matcher->setPreFilterCap(63);
    left_matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
    right_matcher = cv::ximgproc::createRightMatcher(left_matcher);

    wls_filter = cv::ximgproc::createDisparityWLSFilter(left_matcher);
    wls_filter->setLambda(lambda);
    wls_filter->setSigmaColor(sigma);
}

StereoMatcher::~StereoMatcher()
{
}

/**
 * Computes disparity in img1 w.r.t. img2 and returns a wighted-least-squares post filtered disparity_map.
 * @param img1 Image of the stereo pair (usually l.h.s)
 * @param img2 Image of the stereo pair (usually r.h.s)
 * @return Disparity map post filtered using weighted-least-squares
 */
cv::Mat StereoMatcher::disparity(cv::Mat &img1, cv::Mat &img2)
{
    cv::Mat disp1;
    cv::Mat disp2;
    cv::Mat filtered;

    left_matcher->compute(img1, img2, disp1);
    right_matcher->compute(img2, img1, disp2);
    wls_filter->filter(disp1, img1, filtered, disp2);

    cv::Mat scaled;
    filtered.convertTo(scaled, CV_32F, 1.0 / 16.0f);
    return scaled;
}

/**
 * Construct a point cloud generator that uses the given StereoMatcher to produce disparity map
 * and reprojects it to 3D using the given projection matrix Q.
 * @param matcher StereoMatcher used for disparity mapping.
 * @param Q The camera projection matrix used for 3D reprojection.
 */
PointCloudGenerator::PointCloudGenerator(StereoMatcher &matcher, cv::Mat &Q)
        : stereoMatcher_(matcher), Q_(Q)
{
}

PointCloudGenerator::~PointCloudGenerator()
{
}

/**
 * Generate a 3D point cloud from the disparity of a pair of grey-scale stereo images
 * @param left The left image of the stereo pair
 * @param right The right image of the stereo pair
 * @return The 3D point cloud generated from the stereo images
 */
pcl::PointCloud<pcl::PointXYZRGB>::Ptr PointCloudGenerator::generate(cv::Mat& left, cv::Mat& right)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud(new  pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new  pcl::PointCloud<pcl::PointXYZRGB>());

    cv::Mat disparity = stereoMatcher_.disparity(left, right);
    pointcloud->width = static_cast<uint32_t>(disparity.cols);
    pointcloud->height = static_cast<uint32_t>(disparity.rows);
    pointcloud->is_dense = true;

    cv::Mat xyz;
    reprojectImageTo3D(disparity, xyz, Q_, true);
    pcl::PointXYZRGB point;

    for (int i = 0; i < disparity.rows; ++i)
    {
        for (int j = 0; j < disparity.cols; ++j)
        {
            cv::Point3f p = xyz.at<cv::Point3f>(i, j);
            point.x = p.x;
            point.y = p.y;
            point.z = p.z;
            point.b = left.ptr<uchar>(i)[j];
            point.g = left.ptr<uchar>(i)[j];
            point.r = left.ptr<uchar>(i)[j];
            pointcloud->points.push_back(point);
        }
    }

    // voxel grid filter
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud (pointcloud);
    sor.setLeafSize (0.05, 0.05, 0.05);
    sor.filter (*cloud_filtered);

    return  cloud_filtered;
}