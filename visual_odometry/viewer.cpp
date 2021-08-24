#include <opencv2/opencv.hpp>
#include "frame.h"
#include "feature.h"
#include "viewer.h"

const std::string FEATURES_WINDOW_NAME = "Features";
const std::string STEREO_MATCHES_WINDOW_NAME = "Stereo Matches";
const std::string INTER_FRAME_MATCHES_WINDOW_NAME = "Inter Frame Matches";
const std::string DISPARITY_WINDOW_NAME = "Disparity";

Viewer::Viewer() {}

void Viewer::init() {
    cv::namedWindow(FEATURES_WINDOW_NAME);
    cv::namedWindow(STEREO_MATCHES_WINDOW_NAME);
    //cv::namedWindow(INTER_FRAME_MATCHES_WINDOW_NAME);
    //cv::namedWindow(DISPARITY_WINDOW_NAME);
}

void Viewer::view(std::shared_ptr<Frame> previous_frame, std::shared_ptr<Frame> current_frame) {

    cv::Mat img_features;
    cv::Scalar color_g(0, 255, 0), color_b(255, 0, 0), color_r(0, 0, 255);
    cv::drawKeypoints(current_frame->image_left_, current_frame->keypoints_left_, img_features, color_g);
    cv::imshow(FEATURES_WINDOW_NAME, img_features);
    cv::waitKey(1);

    std::vector<cv::KeyPoint> keypoints;
    for(auto point : current_frame->features_left_) {
        keypoints.push_back(cv::KeyPoint(point.get()->point_, 1.f));
    }
    cv::Mat img_matches;
    cv::drawKeypoints(current_frame->image_left_, keypoints, img_matches, color_g);
    cv::imshow(STEREO_MATCHES_WINDOW_NAME, img_matches);
    cv::waitKey(1);
}

