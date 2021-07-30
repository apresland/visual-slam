#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>

#include "stereo_vision.h"

std::string left_file = "/home/dev/tmp/stereo_vision/left.png";
std::string right_file = "/home/dev/tmp/stereo_vision/right.png";

void compute_stereo(cv::Mat& left, cv::Mat& right, cv::Mat &Q)
{
    StereoMatcher stereoMatcher;
    PointCloudGenerator pointCloud(stereoMatcher, Q);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered = pointCloud.generate(left, right);

    pcl::visualization::CloudViewer viewer ("Simple Cloud Viewer");
    viewer.showCloud(cloud_filtered);

    std::cout << "Press [Enter] to continue . . ." << std::endl;
    std::cin.get();
}

int main(int argc, char **argv) {

    // Q matrix (from KITTI dataset)
    double f = 718.856;
    double cx1 = 607.1928;
    double cx2 = 607.1928;
    double cy = 185.2157;
    double Tx = 0.573;

    cv::Mat Q = cv::Mat(4,4, CV_64F, double(0));
    Q.at<double>(0,0) = 1.0;
    Q.at<double>(0,3) = -cx1;
    Q.at<double>(1,1) = 1.0;
    Q.at<double>(1,3) = -cy;
    Q.at<double>(2,3) = f;
    Q.at<double>(3,2) = -1.0/ Tx;
    Q.at<double>(3,3) = ( cx1 - cx2)/ Tx;

    cv::Mat left = cv::imread(left_file, 0);
    cv::Mat right = cv::imread(right_file, 0);

    compute_stereo(left, right, Q);

    return 0;
}
