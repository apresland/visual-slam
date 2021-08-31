#ifndef VISUAL_SLAM_TRACKER_H
#define VISUAL_SLAM_TRACKER_H

#include <opencv2/opencv.hpp>

class Tracker {
public:
    void track(cv::Mat imgL_t0, cv::Mat imgR_t0, cv::Mat imgL_t1, cv::Mat imgR_t1,
               std::vector<cv::Point2f> &ptsL_t0, std::vector<cv::Point2f> &ptsR_t0,
               std::vector<cv::Point2f> &ptsL_t1, std::vector<cv::Point2f> &ptsR_t1,
               std::vector<cv::Point2f> &ptsL_t0_return);
};

#endif //VISUAL_SLAM_TRACKER_H
