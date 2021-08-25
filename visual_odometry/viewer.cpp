#include <opencv2/opencv.hpp>
#include "frame.h"
#include "feature.h"
#include "viewer.h"

const std::string FEATURES_WINDOW_NAME = "Features";
const std::string BUCKETED_FEATURES_WINDOW_NAME = "Bucketed Fetures";
const std::string CIRCULAR_MATCHES_WINDOW_NAME = "Circular Matched Features";

Viewer::Viewer() {}

void Viewer::init() {
    cv::namedWindow(FEATURES_WINDOW_NAME);
    cv::namedWindow(BUCKETED_FEATURES_WINDOW_NAME);
    cv::namedWindow(CIRCULAR_MATCHES_WINDOW_NAME);
    //cv::namedWindow(DISPARITY_WINDOW_NAME);
}

void Viewer::view(std::shared_ptr<Frame> previous_frame, std::shared_ptr<Frame> current_frame, std::vector<cv::Point2f> current_features_) {

    cv::Mat img_features;
    cv::Scalar color_g(0, 255, 0), color_b(255, 0, 0), color_r(0, 0, 255);
    cv::drawKeypoints(current_frame->image_left_, current_frame->keypoints_left_, img_features, color_g);
    cv::imshow(FEATURES_WINDOW_NAME, img_features);
    cv::waitKey(1);

    std::vector<cv::KeyPoint> keypoints;
    for(auto point : current_frame->features_left_) {
        keypoints.push_back(cv::KeyPoint(point.get()->point_, 1.f));
    }
    cv::Mat img_bucketed;
    cv::drawKeypoints(current_frame->image_left_, keypoints, img_bucketed, color_g);
    cv::imshow(BUCKETED_FEATURES_WINDOW_NAME, img_bucketed);
    cv::waitKey(1);

    std::vector<cv::KeyPoint> keypoints2;
    for(auto point : current_features_) {
        keypoints2.push_back(cv::KeyPoint(point, 1.f));
    }
    cv::Mat img_matches;
    cv::drawKeypoints(current_frame->image_left_, keypoints2, img_matches, color_g);
    cv::imshow(CIRCULAR_MATCHES_WINDOW_NAME, img_matches);
    cv::waitKey(1);
}

