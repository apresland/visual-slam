#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include "feature.h"
#include "frontend.h"

Frontend::Frontend() {
    detector_ = cv::FastFeatureDetector::create(20, true);
}

void Frontend::process(std::shared_ptr<Frame> frame) {

    current_frame_ = frame;

    switch (status_) {
        case INITIALIZING:
            initialize();
            break;
        case TRACKING:
            track();
            break;
        case LOST:
            restart();
            break;
    }

    if(viewer_) {
        viewer_->view(previous_frame_, current_frame_);
    }

    previous_frame_ = current_frame_;
}

int Frontend::initialize() {
    detect_features(current_frame_->image_left_);
    status_ = TRACKING;
}

int Frontend::track() {
    detect_features(current_frame_->image_left_);
}

int Frontend::restart() {

}

int Frontend::detect_features(cv::Mat &image) {

    auto sort_predicate = [](cv::KeyPoint const& kp1, cv::KeyPoint const& kp2)-> bool {
        return kp1.response > kp2.response;
    };

    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(image, keypoints);


    for(int x =0; x < NUMBER_GRID_CELL_COLS; ++x) {
        for(int y =0; y < NUMBER_GRID_CELL_ROWS; ++y) {
            feature_grid_[x][y].clear();
        }
    }

    for (auto &keypoint : keypoints) {
        int x, y;
        if(position_in_grid(keypoint, x, y))
            feature_grid_[x][y].push_back(keypoint);
            current_frame_->keypoints_left_.push_back(keypoint);
    }

    for(int x =0; x < NUMBER_GRID_CELL_COLS; ++x) {
        for(int y =0; y < NUMBER_GRID_CELL_ROWS; ++y) {
            unsigned int count = 0;
            std::sort(feature_grid_[x][y].begin(), feature_grid_[x][y].end(), sort_predicate);
            for (auto &keypoint : feature_grid_[x][y]) {
                if (count >= MAX_FEATURES_PER_CELL) continue;
                std::shared_ptr<Feature> feature = std::make_shared<Feature>(keypoint.pt);
                current_frame_->features_left_.push_back(feature);
                ++count;
            }
        }
    }

}

bool Frontend::position_in_grid(cv::KeyPoint &keypoint, int &grid_pos_x, int &grid_pos_y) {
    float grid_cell_width_inverse = static_cast<float>(NUMBER_GRID_CELL_COLS) / (current_frame_->image_left_.cols);
    float grid_cell_height_inverse = static_cast<float>(NUMBER_GRID_CELL_ROWS) / (current_frame_->image_left_.rows);
    grid_pos_x = round((keypoint.pt.x)*grid_cell_width_inverse);
    grid_pos_y = round((keypoint.pt.y)*grid_cell_height_inverse);
    if(grid_pos_x<0 || grid_pos_x >= NUMBER_GRID_CELL_COLS || grid_pos_y < 0 || grid_pos_y >= NUMBER_GRID_CELL_ROWS)
        return false;

    return true;
}

