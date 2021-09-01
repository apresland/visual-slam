#ifndef VISUAL_SLAM_MAP_H
#define VISUAL_SLAM_MAP_H
#include <mutex>
#include <memory>
#include <map>
#include "landmark.h"
#include "frame.h"

class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Map(){}

    void insert_keyframe(std::shared_ptr<Frame> keyframe);
    void insert_landmark(std::shared_ptr<Landmark> landmark);

    std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes();
    std::unordered_map<unsigned long, std::shared_ptr<Landmark>> landmarks();

    std::mutex data_mutex_;
    std::unordered_map<unsigned long, std::shared_ptr<Landmark>> landmarks_;
    std::unordered_map<unsigned long, std::shared_ptr<Landmark>> active_landmarks_;
    std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes_;
    std::unordered_map<unsigned long, std::shared_ptr<Frame>> active_keyframes_;
    std::shared_ptr<Frame> current_frame_ {nullptr};

    // settings
    int num_active_keyframes_ = 7;
};

#endif //VISUAL_SLAM_MAP_H
