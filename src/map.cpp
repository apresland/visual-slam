#include <utility>
#include "map.h"

void Map::insert_keyframe(std::shared_ptr<Frame> keyframe) {
    current_frame_ = keyframe;
    if (keyframes_.find(keyframe->id_) == keyframes_.end()) {
        keyframes_.insert(make_pair(keyframe->id_, keyframe));
        active_keyframes_.insert(make_pair(keyframe->id_, keyframe));
    } else {
        keyframes_[keyframe->id_] = keyframe;
        active_keyframes_[keyframe->id_] = keyframe;
    }
}

void Map::insert_landmark(std::shared_ptr<MapPoint> landmark) {
    if (landmarks_.find(landmark->id_) == landmarks_.end()) {
        landmarks_.insert( LandmarksType::value_type(landmark->id_, landmark));
        active_landmarks_.insert(LandmarksType::value_type(landmark->id_, landmark));
    } else {
        landmarks_.at(landmark->id_) = landmark;
        active_landmarks_.at(landmark->id_) = landmark;
    }
}

std::unordered_map<unsigned long, std::shared_ptr<Frame>> Map::keyframes() {
    return keyframes_;
}

Map::LandmarksType Map::landmarks() {
    return landmarks_;
}

