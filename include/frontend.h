#ifndef VISUAL_SLAM_FRONTEND_H
#define VISUAL_SLAM_FRONTEND_H

#include <memory>
#include "sophus/se3.hpp"
#include "camera.h"
#include "frame.h"
#include "detector.h"
#include "matcher.h"
#include "tracker.h"
#include "triangulator.h"
#include "estimation.h"
#include "backend.h"
#include "map.h"
#include "viewer.h"

constexpr unsigned int MIN_FEATURE_COUNT = 500;
constexpr float IMAGE_SCALE_FACTOR = 1.0;

class Frontend {

public:
    enum Status {INITIALIZING, LOST, TRACKING};
public:
    Frontend();
    void update(std::shared_ptr<Frame> frame);
    void set_map(std::shared_ptr<Map> map) {
        map_ = map;
    }
    void set_backend(std::shared_ptr<Backend> backend) {
        backend_ = backend;
    }
    void set_viewer(std::shared_ptr<Viewer> viewer) {
        viewer_ = viewer;
    }
    void set_matcher(std::shared_ptr<Matcher> matcher) {
        matcher_ = matcher;
    }
    void set_tracker(std::shared_ptr<Tracker> tracker) {
        tracker_ = tracker;
    }
    void set_triangulator(std::shared_ptr<Triangulator> triangulator) {
        triangulator_ = triangulator;
    }
    void set_estimation(std::shared_ptr<Estimation> estimation) {
        estimation_ = estimation;
    }

    void set_cameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }

private:
    int initialize(std::shared_ptr<Frame> frame);
    int process(std::shared_ptr<Frame> frame_previous, std::shared_ptr<Frame> frame_current, std::shared_ptr<Frame> frame_next);
    int restart();

    void initialize_map(std::shared_ptr<Frame> frame);
    void insert_keyframe(std::shared_ptr<Frame> frame);

    Detector detector_;
    std::shared_ptr<Matcher> matcher_{};
    std::shared_ptr<Tracker> tracker_{};
    std::shared_ptr<Triangulator> triangulator_{};
    std::shared_ptr<Estimation> estimation_{};
    std::shared_ptr<Map> map_ {nullptr};
    std::shared_ptr<Backend> backend_ {nullptr};
    std::shared_ptr<Viewer> viewer_{nullptr};
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Frame> frame_previous_{nullptr};
    std::shared_ptr<Frame> frame_current_{nullptr};
    std::shared_ptr<Frame> frame_next_{nullptr};
    bool has_new_detections_{false};
    size_t feature_id_{0};
    size_t landmark_id_{0};
    size_t frame_id_{0};

    Status status_{INITIALIZING};
};

#endif //VISUAL_SLAM_FRONTEND_HS
