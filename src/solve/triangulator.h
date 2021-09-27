#ifndef VISUAL_SLAM_TRIANGULATION_H
#define VISUAL_SLAM_TRIANGULATION_H
#include <opencv2/opencv.hpp>
#include <memory>
#include <sensor/camera.h>
#include <map/map.h>
#include "context.h"

struct Frame;
class Triangulator {
public:
    Triangulator(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right)
        : camera_left_(camera_left), camera_right_(camera_right) {}
    void triangulate(Context &context, bool is_initialize);
    void setMap(std::shared_ptr<Map> map) { map_ = map;}

private:
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Map> map_{nullptr};
    size_t landmark_id_{0};

};

#endif //VISUAL_SLAM_TRIANGULATION_H
