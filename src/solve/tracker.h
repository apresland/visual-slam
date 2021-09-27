#ifndef VISUAL_SLAM_TRACKER_H
#define VISUAL_SLAM_TRACKER_H

#include <memory>
#include "context.h"

struct Frame;
class Tracker {
public:
Tracker() {}
void track(Context &context);
};

#endif //VISUAL_SLAM_TRACKER_H
