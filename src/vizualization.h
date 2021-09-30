#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>
#include <fstream>
#include "sophus/se3.hpp"
#include "sensor/frame.h"
#include "context.h"

struct Vizualization {
    static void prepareKeypointsVisual(const Context &context, cv::Mat& output_image);
    static void prepareFeaturesVisual(const Context &context, cv::Mat& output_image);
    static void prepareOpticalFlowVizual(const Context &context, cv::Mat& output_image);
};
#endif //VISUAL_SLAM_VIEWER_H
