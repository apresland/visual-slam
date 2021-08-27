
#include "detector.h"

Detector::Detector() {
    detector_ = cv::FastFeatureDetector::create(20, true);
}

static const auto keypoint_response_comparitor = [](cv::KeyPoint const& kp1, cv::KeyPoint const& kp2)-> bool {
    return abs(kp1.response) > abs(kp2.response);
};

bool Detector::detect(const cv::Mat &image, std::vector<cv::Point2f> &features) {

    int x, y;

    // FAST keypoint detection and presort on quality
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(image, keypoints);
    std::sort(keypoints.begin(), keypoints.end(), keypoint_response_comparitor);

    // Bucket with MAX_FEATURES_PER_CELL on presorted detections
    std::vector<cv::KeyPoint> keypoint_subset;
    unsigned int keypoint_cell_count[NUMBER_GRID_CELL_COLS][NUMBER_GRID_CELL_ROWS];
    for (auto &keypoint : keypoints) {
        if(position_in_grid(image, keypoint.pt, x, y))
            if (MAX_FEATURES_PER_CELL > keypoint_cell_count[x][y])
                ++keypoint_cell_count[x][y];
                keypoint_subset.push_back(keypoint);
                features.push_back(keypoint.pt);
    }
}

bool Detector::position_in_grid(const cv::Mat &image, cv::Point2f &point, int &x, int &y) {
    x = round((point.x) * static_cast<float>(NUMBER_GRID_CELL_COLS) / (image.cols));
    y = round((point.y) * static_cast<float>(NUMBER_GRID_CELL_ROWS) / (image.rows));
    return (x >=0 && x < NUMBER_GRID_CELL_COLS && y >= 0 && y < NUMBER_GRID_CELL_ROWS);
}
