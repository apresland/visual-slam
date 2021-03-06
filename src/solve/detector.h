#ifndef VISUAL_SLAM_DETECTOR_H
#define VISUAL_SLAM_DETECTOR_H

#include <memory>
#include <opencv2/opencv.hpp>
#include <sensor/frame.h>
#include <sensor/feature.h>
#include "context.h"

constexpr unsigned int GRID_CELL_SIZE = 15;

class Detector {
public:
    Detector();
    bool detect(Context &context);
private:
    bool positionInGrid(const cv::Mat &image, cv::Point2f &point, int &grid_pos_x, int &grid_pos_y);
    cv::Ptr<cv::FeatureDetector> detector_;
    int NUMBER_GRID_CELL_COLS;
    int NUMBER_GRID_CELL_ROWS;
};

#endif //VISUAL_SLAM_DETECTOR_H
