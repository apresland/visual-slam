#ifndef VISUAL_SLAM_BACKEND_H
#define VISUAL_SLAM_BACKEND_H

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "camera.h"
#include "map.h"

class Backend {
public:

    Backend();

    void setCameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }
    void setMap(std::shared_ptr<Map> map) { map_ = map; }

    void execute();
    void terminate();
    void optimize(Map::KeyframesType keyframes,
                  Map::LandmarksType landmarks);

    void updateMap();

private:
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Map> map_ {nullptr};

    std::thread thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> running_;
};

#endif //VISUAL_SLAM_BACKEND_H
