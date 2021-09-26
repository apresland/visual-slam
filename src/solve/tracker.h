#ifndef VISUAL_SLAM_TRACKER_H
#define VISUAL_SLAM_TRACKER_H

#include <memory>

struct Frame;
class Tracker {
public:
Tracker() {}
void track(std::shared_ptr<Frame> frame_previous, std::shared_ptr<Frame> frame_current);
};

#endif //VISUAL_SLAM_TRACKER_H
