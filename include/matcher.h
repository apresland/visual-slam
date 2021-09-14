#ifndef VISUAL_SLAM_MATCHER_H
#define VISUAL_SLAM_MATCHER_H

#include <opencv2/opencv.hpp>
#include "frame.h"

class Matcher {
public:
    void match(std::shared_ptr<Frame> frame);
    void match_circular(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1);
};

#endif //VISUAL_SLAM_MATCHER_H
