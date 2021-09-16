#ifndef VISUAL_SLAM_MATCHER_H
#define VISUAL_SLAM_MATCHER_H

#include <opencv2/opencv.hpp>
#include "frame.h"
#include "viewer.h"

class Matcher {
public:
    Matcher() {}
    void match(std::shared_ptr<Frame> frame);
    void match(std::shared_ptr<Frame> frame_current, std::shared_ptr<Frame> frame_next);
};

#endif //VISUAL_SLAM_MATCHER_H
