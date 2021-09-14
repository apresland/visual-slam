#ifndef VISUAL_SLAM_TRACKER_H
#define VISUAL_SLAM_TRACKER_H

#include <opencv2/opencv.hpp>
#include "frame.h"

class Tracker {
public:
    void track(std::shared_ptr<Frame> frame);
    void track(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1);
};

#endif //VISUAL_SLAM_TRACKER_H
