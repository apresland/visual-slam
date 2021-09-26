#ifndef VISUAL_SLAM_MAP_H
#define VISUAL_SLAM_MAP_H
#include <mutex>
#include <memory>
#include <unordered_map>
#include <sensor/mappoint.h>
#include <sensor/frame.h>

class Map {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Map(){}

    typedef std::unordered_map<unsigned long, std::shared_ptr<MapPoint>> LandmarksType;
    typedef std::unordered_map<unsigned long, std::shared_ptr<Frame>> KeyframesType;

    void insertKeyframe(std::shared_ptr<Frame> keyframe);
    void insertLandmark(std::shared_ptr<MapPoint> landmark);
    void removeKeyframe();
    void clean_map();

    std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes();
    LandmarksType landmarks();

    LandmarksType landmarks_;
    LandmarksType active_landmarks_;
    KeyframesType keyframes_;
    KeyframesType active_keyframes_;
    std::shared_ptr<Frame> current_frame_ {nullptr};

    // settings
    int num_active_keyframes_ = 10;
};

#endif //VISUAL_SLAM_MAP_H
