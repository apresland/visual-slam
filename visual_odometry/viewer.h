#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>
#include <fstream>


class Viewer {
public:
    Viewer();
    void init();
    void load_poses();
    void display_features(const cv::Mat &image, const std::vector<cv::Point2f> points_left_t1);
    void display_tracking(const cv::Mat &image_left_t1, std::vector<cv::Point2f> points_left_t0, std::vector<cv::Point2f> points_left_t1);
    void display_trajectory(cv::Mat& pose, unsigned int true_pose_id);

private:
    std::vector<cv::Mat> ground_truth_poses_;
    cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
};
#endif //VISUAL_SLAM_VIEWER_H
