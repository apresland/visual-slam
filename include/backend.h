#ifndef VISUAL_SLAM_BACKEND_H
#define VISUAL_SLAM_BACKEND_H

#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include "camera.h"
#include "map.h"
#include "optimization.h"

class Backend {
public:

    Backend();

    void set_cameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }
    void set_map(std::shared_ptr<Map> map) { map_ = map; }

    void execute();
    void terminate();
    void optimize(std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes,
                  std::unordered_map<unsigned long, Landmark> landmarks);

private:
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Map> map_ {nullptr};

    std::thread thread_;
    std::mutex data_mutex_;

    std::condition_variable map_update_;
    std::atomic<bool> running_;

    Optimization optimization_;
};

#endif //VISUAL_SLAM_BACKEND_H
