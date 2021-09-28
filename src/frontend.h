#ifndef VISUAL_SLAM_FRONTEND_H
#define VISUAL_SLAM_FRONTEND_H

#include <memory>
#include "sophus/se3.hpp"
#include <sensor/camera.h>
#include <sensor/frame.h>
#include <solve/detector.h>
#include <solve/tracker.h>
#include <solve/triangulator.h>
#include <solve/estimation.h>
#include <solve/matcher.h>
#include <map/map.h>
#include "context.h"
#include "backend.h"
#include "viewer.h"

constexpr unsigned int MIN_FEATURE_COUNT = 500;
constexpr float IMAGE_SCALE_FACTOR = 1.0;

class Frontend {

public:
    enum Status {INITIALIZING, LOST, TRACKING};
public:
    Frontend();
    void pushback(cv::Mat &image_left, cv::Mat &image_right);
    void setMap(std::shared_ptr<Map> map) {
        map_ = map;
    }
    void setBackend(std::shared_ptr<Backend> backend) {
        backend_ = backend;
    }
    void setViewer(std::shared_ptr<Viewer> viewer) {
        viewer_ = viewer;
    }
    void setMatcher(std::shared_ptr<Matcher> matcher) {
        matcher_ = matcher;
    }
    void setTracker(std::shared_ptr<Tracker> tracker) {
        tracker_ = tracker;
    }
    void setTriangulator(std::shared_ptr<Triangulator> triangulator) {
        triangulator_ = triangulator;
    }
    void setEstimation(std::shared_ptr<Estimation> estimation) {
        estimation_ = estimation;
    }

    void setCameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }

private:
    int initialize();
    int process();
    int restart();

    void insertKeyframe(std::shared_ptr<Frame> &frame);

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

    Context context_;

    Status status_{INITIALIZING};
};

#endif //VISUAL_SLAM_FRONTEND_HS
