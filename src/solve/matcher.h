#ifndef VISUAL_SLAM_MATCHER_H
#define VISUAL_SLAM_MATCHER_H

#include <opencv2/opencv.hpp>
#include "context.h"
#include <sensor/frame.h>

class Matcher {
public:
    Matcher() {}
    void match_stereo(Context &context);
    void match_quadro(Context &context);
};

#endif //VISUAL_SLAM_MATCHER_H
