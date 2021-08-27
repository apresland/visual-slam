#ifndef VISUAL_SLAM_FRONTEND_H
#define VISUAL_SLAM_FRONTEND_H

#include <memory>
#include "camera.h"
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
    void update(const cv::Mat &image_left, const cv::Mat &image_right);
    void set_viewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
    void set_cameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }

private:
    int initialize(const cv::Mat &image_left_t1);
    int process(const cv::Mat &image_left_t0, const cv::Mat &image_right_t0,
                const cv::Mat &image_left_t1, const cv::Mat &image_right_t1);
    int restart();

    void estimate_motion(const std::vector<cv::Point2f>&  image_points_2d,
                         const cv::Mat& object_points_3d,
                         const cv::Mat K,
                         const cv::Mat& R,
                         const cv::Mat& t);

    void update_pose(cv::Mat& pose,
                     const cv::Mat& R,
                     const cv::Mat& t);

    Detector detector_;
    Tracker tracker_;
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Viewer> viewer_ = nullptr;

    cv::Mat image_left_t0;
    cv::Mat image_right_t0;
    std::vector<cv::Point2f> features_;
    cv::Mat pose_ = cv::Mat::eye(4, 4, CV_64F);

    size_t frame_id_;
    Status status_{INITIALIZING};
};

#endif //VISUAL_SLAM_FRONTEND_HS
