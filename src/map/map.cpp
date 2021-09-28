#include <utility>
#include <map/map.h>

void Map::insertKeyframe(std::shared_ptr<Frame> keyframe) {
    current_frame_ = keyframe;
    if (keyframes_.find(keyframe->getID()) == keyframes_.end()) {
        keyframes_.insert(make_pair(keyframe->getID(), keyframe));
        active_keyframes_.insert(make_pair(keyframe->getID(), keyframe));
    } else {
        keyframes_[keyframe->getID()] = keyframe;
        active_keyframes_[keyframe->getID()] = keyframe;
    }
    if (active_keyframes_.size() > num_active_keyframes_) {
        removeKeyframe();
    }
}

void Map::insertLandmark(std::shared_ptr<MapPoint> landmark) {
    if (landmarks_.find(landmark->getID()) == landmarks_.end()) {
        landmarks_.insert( LandmarksType::value_type(landmark->getID(), landmark));
        active_landmarks_.insert(LandmarksType::value_type(landmark->getID(), landmark));
    } else {
        landmarks_.at(landmark->getID()) = landmark;
        active_landmarks_.at(landmark->getID()) = landmark;
    }
}

std::unordered_map<unsigned long, std::shared_ptr<Frame>> Map::keyframes() {
    return keyframes_;
}

Map::LandmarksType Map::landmarks() {
    return landmarks_;
}

void Map::removeKeyframe() {
    if (current_frame_ == nullptr) return;
    double max_dis = 0, min_dis = 9999;
    double max_kf_id = 0, min_kf_id = 0;
    auto Twc = current_frame_->getPose().inverse();
    for (auto& kf : active_keyframes_) {
        if (kf.second == current_frame_) continue;
        auto dis = (kf.second->getPose() * Twc).log().norm();
        if (dis > max_dis) {
            max_dis = dis;
            max_kf_id = kf.first;
        }
        if (dis < min_dis) {
            min_dis = dis;
            min_kf_id = kf.first;
        }
    }

    const double min_dis_th = 0.2;
    std::shared_ptr<Frame> frame_to_remove {nullptr};
    if (min_dis < min_dis_th) {
        frame_to_remove = keyframes_.at(min_kf_id);
    } else {
        frame_to_remove = keyframes_.at(max_kf_id);
    }

    active_keyframes_.erase(frame_to_remove->getID());
    clean_map();
}

void Map::clean_map() {
    int cnt_landmark_removed = 0;
    for (auto iter = active_landmarks_.begin();
         iter != active_landmarks_.end();) {
        if (iter->second->getObservedTimes() == 0) {
            iter = active_landmarks_.erase(iter);
            cnt_landmark_removed++;
        } else {
            ++iter;
        }
    }
    std::cout << "Removed " << cnt_landmark_removed << " active landmarks" << std::endl;
}

