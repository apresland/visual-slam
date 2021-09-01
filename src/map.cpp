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

    //if (active_keyframes_.size() > num_active_keyframes_) {
        //RemoveOldKeyframe();
    //}
}
void Map::insert_landmark(std::shared_ptr<Landmark> landmark) {
    if (landmarks_.find(landmark->id_) == landmarks_.end()) {
        landmarks_.insert(make_pair(landmark->id_, landmark));
        active_landmarks_.insert(std::make_pair(landmark->id_, landmark));
    } else {
        landmarks_[landmark->id_] = landmark;
        active_landmarks_[landmark->id_] = landmark;
    }
}

std::unordered_map<unsigned long, std::shared_ptr<Frame>> Map::keyframes() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return keyframes_;
}

std::unordered_map<unsigned long, std::shared_ptr<Landmark>> Map::landmarks() {
    std::unique_lock<std::mutex> lck(data_mutex_);
    return landmarks_;
}

