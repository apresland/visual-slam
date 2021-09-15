#ifndef VISUAL_SLAM_MATCHER_H
#define VISUAL_SLAM_MATCHER_H

#include <opencv2/opencv.hpp>
#include "frame.h"
#include "viewer.h"

class Matcher {
public:
    Matcher() {}
    void match(std::shared_ptr<Frame> frame);
    void match(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1);
    void track(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1);
    void set_viewer(std::shared_ptr<Viewer> viewer) {viewer_ = viewer;}
    std::shared_ptr<Viewer> viewer_;
};

#endif //VISUAL_SLAM_MATCHER_H
