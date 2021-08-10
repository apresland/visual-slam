#ifndef VISUAL_SLAM_VIEWER_H
#define VISUAL_SLAM_VIEWER_H

#include <memory>

class Viewer {
public:
    Viewer();
    void init();
    void view(std::shared_ptr<Frame> previous_frame, std::shared_ptr<Frame> current_frame);
};
#endif //VISUAL_SLAM_VIEWER_H
