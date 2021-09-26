#include "backend.h"
#include <optimize/optimization.h>

Backend::Backend() {
    running_.store(true);
    thread_ = std::thread(std::bind(&Backend::execute, this));
}

void Backend::execute() {
    while (running_.load()) {
        std::cout << "[INFO] Backend::execute - executing optimization"<< std::endl;
        std::unique_lock<std::mutex> lock(data_mutex_);
        map_update_.wait(lock);
        Map::KeyframesType active_keyframes = map_->active_keyframes_;
        Map::LandmarksType active_landmarks = map_->active_landmarks_;
        optimize(active_keyframes, active_landmarks);
    }
}

void Backend::updateMap() {
    std::unique_lock<std::mutex> lock(data_mutex_);
    map_update_.notify_one();
    std::cout << "[INFO] Backend::updateMap - issued update notification"<< std::endl;
}

void Backend::optimize(Map::KeyframesType keyframes,
                       Map::LandmarksType landmarks) {
    std::shared_ptr<Optimization> optimizer = std::make_shared<Optimization>();
    optimizer->optimize(keyframes, landmarks, camera_left_->K());
}

void Backend::terminate() {
    running_.store(false);
    map_update_.notify_one();
    thread_.join();
}
