//
// Created by andy on 7/29/21.
//

#ifndef VISUAL_SLAM_STEREO_VISION_H
#define VISUAL_SLAM_STEREO_VISION_H

#include <string>
#include <opencv4/opencv2/opencv.hpp>
#include "opencv2/ximgproc/disparity_filter.hpp"
#include <opencv4/opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/**
 * A stereo matcher to generate filtered disparity maps.
 */
class StereoMatcher {
public:
    StereoMatcher();
    ~StereoMatcher();

    cv::Mat disparity(cv::Mat &left, cv::Mat &right);

private:

    int wsize = 13;
    int max_disp = 16 * 10;
    double lambda = 10000.0;
    double sigma = 1.0;

    cv::Ptr<cv::StereoSGBM> left_matcher;
    cv::Ptr<cv::StereoMatcher> right_matcher;
    cv::Ptr<cv::ximgproc::DisparityWLSFilter> wls_filter;
};

/**
 * Generate a pointcloud from left and right images given the camera projection matrix Q.
 * Internally generates a disparity map from the stereo image pair, reprojects it into 3D space
 * and convert into a voxel grid
 */
class PointCloudGenerator {
public:
    PointCloudGenerator(StereoMatcher &matcher, cv::Mat &Q);
    ~PointCloudGenerator();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr generate(cv::Mat& left, cv::Mat& right);
private:
    cv::Mat& Q_;
    StereoMatcher& stereoMatcher_;
};
#endif //VISUAL_SLAM_STEREO_VISION_H
