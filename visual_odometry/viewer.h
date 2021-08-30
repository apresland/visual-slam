#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>
#include <fstream>
#include "sophus/se3.hpp"

class Viewer {
public:
    Viewer();
    void init();
    void load_poses();
    void display_features(const cv::Mat &image, const std::vector<cv::Point2f> points_left_t1);
    void display_tracking(const cv::Mat &image_left_t1, std::vector<cv::Point2f> points_left_t0, std::vector<cv::Point2f> points_left_t1);
    void display_trajectory(Sophus::SE3d T_c_w_, unsigned int true_pose_id);

private:
    std::vector<cv::Mat> ground_truth_poses_;
    cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
};
#endif //VISUAL_SLAM_VIEWER_H
