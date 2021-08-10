#include <opencv2/opencv.hpp>
#include "frame.h"
#include "viewer.h"

const std::string FEATURES_WINDOW_NAME = "Features";
const std::string MATCHES_WINDOW_NAME = "Matches";

Viewer::Viewer() {}

void Viewer::init() {
    cv::namedWindow(FEATURES_WINDOW_NAME);
    cv::namedWindow(MATCHES_WINDOW_NAME);
}

void Viewer::view(std::shared_ptr<Frame> previous_frame, std::shared_ptr<Frame> current_frame) {
    cv::Mat img_features;
    cv::Scalar color_g(0, 255, 0), color_b(255, 0, 0), color_r(0, 0, 255);
    cv::drawKeypoints(current_frame->image_left_, current_frame->keypoints_, img_features, color_g);
    cv::imshow(FEATURES_WINDOW_NAME, img_features);
    cv::waitKey(1);
    if (!current_frame->matches_.empty()) {
        cv::Mat img_matches;
        cv::drawMatches(previous_frame->image_left_, previous_frame->keypoints_,
                        current_frame->image_left_, current_frame->keypoints_,
                        current_frame->matches_, img_matches);
        cv::imshow(MATCHES_WINDOW_NAME, img_matches);
        cv::waitKey(1);
    }
}

