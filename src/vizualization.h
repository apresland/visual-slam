#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>
#include <fstream>
#include "sophus/se3.hpp"
#include "sensor/frame.h"
#include "context.h"

struct Vizualization {
    cv::Mat prepareFeaturesVisual(const Context &context);
    cv::Mat prepareOpticalFlowVizual(const Context &context);
};
#endif //VISUAL_SLAM_VIEWER_H
