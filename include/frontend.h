#ifndef VISUAL_SLAM_FRONTEND_H
#define VISUAL_SLAM_FRONTEND_H

#include <memory>
#include "sophus/se3.hpp"
#include "camera.h"
#include "frame.h"
#include "detector.h"
#include "tracker.h"
#include "viewer.h"

constexpr unsigned int MIN_FEATURE_COUNT = 1000;
constexpr float IMAGE_SCALE_FACTOR = 1.0;

class Frontend {

public:
    enum Status {INITIALIZING, LOST, TRACKING};
public:
    Frontend();
    void update(std::shared_ptr<Frame> frame);
    void set_viewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
    void set_cameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }

private:
    int initialize(std::shared_ptr<Frame> frame_t1);
    int process(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1);
    int restart();

    int estimate_pose_3d2d_ransac(const std::vector<cv::Point2f>&  image_points_2d,
                                  const cv::Mat& object_points_3d,
                                  const cv::Mat K,
                                  Sophus::SE3d &T_c_w);

    Detector detector_;
    Tracker tracker_;
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Viewer> viewer_ = nullptr;

    Sophus::SE3d T_c_w_ = Sophus::SE3d();

    std::shared_ptr<Frame> frame_t0_{nullptr};
    std::shared_ptr<Frame> frame_t1_{nullptr};
    size_t frame_id_;

    Status status_{INITIALIZING};
};

#endif //VISUAL_SLAM_FRONTEND_HS
