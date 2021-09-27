#include "viewer.h"
#include "detector.h"

Detector::Detector() {
    detector_ = cv::FastFeatureDetector::create(20, true);
}

static const auto keypoint_response_comparitor = [](cv::KeyPoint const& kp1, cv::KeyPoint const& kp2)-> bool {
    return abs(kp1.response) > abs(kp2.response);
};

bool Detector::detect(Context &context) {

    if(!context.frame_current_) return false;

    std::cout << "[INFO] Detector::detect - creating new keypoints" << std::endl;

    NUMBER_GRID_CELL_COLS = std::ceil(static_cast<double>(context.frame_current_->image_left_.cols)/GRID_CELL_SIZE);
    NUMBER_GRID_CELL_ROWS = std::ceil(static_cast<double >(context.frame_current_->image_left_.rows)/GRID_CELL_SIZE);
    std::vector<bool> occupancy_grid(NUMBER_GRID_CELL_COLS*NUMBER_GRID_CELL_ROWS, false);

    // occupancy grid initialize with existing features
    std::vector<cv::Point2f> previous_keypoints = context.frame_current_->getPointsLeft();
    for (int i =0; i < previous_keypoints.size(); i++)
    {
        int x = previous_keypoints[i].x;
        int y = previous_keypoints[i].y;
        int index = static_cast<int>(y/GRID_CELL_SIZE)*NUMBER_GRID_CELL_COLS + static_cast<int>(x/GRID_CELL_SIZE);
        occupancy_grid.at(index) = true;
    }


    // FAST keypoint detection and presort on quality
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(context.frame_current_->image_left_, keypoints);

    // sort detections by quality of response
    std::sort(keypoints.begin(), keypoints.end(), keypoint_response_comparitor);

    // fill unoccupied cells with best detections
    int new_feature_count = 0;
    for (int i =0; i < keypoints.size(); i++)
    {
        int x = keypoints[i].pt.x;
        int y = keypoints[i].pt.y;
        int index = static_cast<int>(y/GRID_CELL_SIZE)*NUMBER_GRID_CELL_COLS + static_cast<int>(x/GRID_CELL_SIZE);
        if ( ! occupancy_grid.at(index)) {
            context.frame_current_->features_left_.push_back(std::make_shared<Feature>(context.frame_current_, keypoints[i].pt));
            occupancy_grid.at(index) = true;
            ++new_feature_count;
        }
    }

    if (context.viewer_) {
        context.viewer_->displayFeatures(context);
    }

    std::cout << "[INFO] Detector::detected - keypoints: total { " << context.frame_current_->features_left_.size() <<  " : new " << new_feature_count << " }" << std::endl;
}
