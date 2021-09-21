#include "backend.h"

Backend::Backend() {
    running_.store(true);
    thread_ = std::thread(std::bind(&Backend::execute, this));
}

void Backend::execute() {
    while (running_.load()) {
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);
        Map::KeyframesType active_keyframes = map_->active_keyframes_;
        Map::LandmarksType active_landmarks = map_->active_landmarks_;
        optimize(active_keyframes, active_landmarks);
    }

}

void Backend::update_map() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
}

void Backend::optimize(Map::KeyframesType keyframes,
              Map::LandmarksType landmarks) {
    optimization_.compute(keyframes, landmarks, camera_left_->K());
}

void Backend::terminate() {
    running_.store(false);
    map_update_.notify_one();
    thread_.join();
}
