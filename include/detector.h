#ifndef VISUAL_SLAM_DETECTOR_H
#define VISUAL_SLAM_DETECTOR_H

#include <opencv2/opencv.hpp>
#include "feature.h"

constexpr unsigned int NUMBER_GRID_CELL_ROWS = 8;
constexpr unsigned int NUMBER_GRID_CELL_COLS = 16;
constexpr unsigned int MAX_FEATURES_PER_CELL = 10;

class Detector {
public:
    Detector();
    bool detect(const cv::Mat &image, std::vector<cv::Point2f> &points);
private:
    bool position_in_grid(const cv::Mat &image, cv::Point2f &point, int &grid_pos_x, int &grid_pos_y);
    cv::Ptr<cv::FeatureDetector> detector_;
};

#endif //VISUAL_SLAM_DETECTOR_H
