#include <algorithm>
#include <opencv2/opencv.hpp>
#include "frontend.h"

Frontend::Frontend() {
    detector_ = cv::ORB::create(1000);
    descriptor_ = cv::ORB::create();
    matcher_ = cv::BFMatcher::create(cv::NORM_HAMMING, true);
}

void Frontend::process(std::shared_ptr<Frame> frame) {

    current_frame_ = frame;
    detect_features();
    if(previous_frame_) {
        match_features();
    }

    if(viewer_) {
        viewer_->view(previous_frame_, current_frame_);
    }

    previous_frame_ = current_frame_;
}

/**
 * Feature detection in the current frame using cv::FeatureDetector.
 * Use the concrete cv::FeatureDetector supplied at construction
 * to dectect features in the left image of the stereo pair.
 * @return The number of features detected.
 */
int Frontend::detect_features() {
    int count_keypoints = 0;
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(current_frame_->image_left_, keypoints);
    for (auto &kp : keypoints) {
        current_frame_->keypoints_.push_back(kp);
        ++count_keypoints;
    }
    descriptor_->compute(current_frame_->image_left_, keypoints, current_frame_->descriptors_);
    return count_keypoints;
}

int Frontend::match_features() {
    int count_keypoints = 0;
    std::vector<cv::DMatch> matches;
    matcher_->match(previous_frame_->descriptors_, current_frame_->descriptors_, matches);

    auto min_max = minmax_element(matches.begin(), matches.end(), [](const cv::DMatch &lhs, const cv::DMatch &rhs) {
        return lhs.distance < rhs.distance;
    });

    auto min_element = min_max.first;
    auto max_element = min_max.second;

    for (auto &m : matches) {
        if( m.distance <= std::max(2.0 * min_element->distance, 30.0) ) {
            current_frame_->matches_.push_back(m);
            ++count_keypoints;
        }
    }
}
