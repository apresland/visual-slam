#include <iostream>
#include <vector>
#include <opencv2/opencv.hpp>
#include <sensor/frame.h>
#include "tracker.h"

void Tracker::track(std::shared_ptr<Frame> frame_previous, std::shared_ptr<Frame> frame_current) {
    std::cout << "[INFO] Tracker::track - tracking " << frame_previous->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_2d_left = frame_previous->getPointsLeft();
    std::vector<cv::Point2f> frame_t1_points_2d_left = frame_previous->getPointsLeft();

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    std::vector<uchar> status;

    calcOpticalFlowPyrLK(frame_previous->image_left_, frame_current->image_left_, frame_t0_points_2d_left, frame_t1_points_2d_left, status, err, window_size, 3, term_criteria, 0, 0.001);

    frame_current->features_left_.clear();
    for(int i=0; i < status.size(); i++) {
        if(status[i]) {
            cv::Point2f p2d_t1 = frame_t1_points_2d_left[i];
            std::shared_ptr<Feature> feature_t1 = std::make_shared<Feature>(frame_current, p2d_t1);
            feature_t1->landmark_ = frame_previous->features_left_[i]->landmark_;
            frame_current->features_left_.push_back(feature_t1);
        }
    }

    std::cout << "[INFO] Tracker::track - tracked " << frame_current->features_left_.size() << " 2D points" << std::endl;
}
