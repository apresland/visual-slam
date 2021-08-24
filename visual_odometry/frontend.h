#ifndef VISUAL_SLAM_FRONTEND_H
#define VISUAL_SLAM_FRONTEND_H

#include <memory>
#include "frame.h"
#include "camera.h"
#include "viewer.h"

constexpr unsigned int NUMBER_GRID_CELL_ROWS = 8;
constexpr unsigned int NUMBER_GRID_CELL_COLS = 16;
constexpr unsigned int MAX_FEATURES_PER_CELL = 10;

class Frontend {

public:
    enum Status {INITIALIZING, LOST, TRACKING};
public:
    Frontend();
    void process(std::shared_ptr<Frame> frame);
    void set_viewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }
    void set_cameras(std::shared_ptr<Camera> camera_left, std::shared_ptr<Camera> camera_right) {
        camera_left_ = camera_left;
        camera_right_ = camera_right;
    }

private:
    int initialize();
    int track();
    int restart();
    int detect_features(cv::Mat &image);
    void assign_features_to_grid(std::vector<cv::KeyPoint> keypoints);
    bool position_in_grid(cv::KeyPoint &keypoint, int &grid_pos_x, int &grid_pos_y);

    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> previous_frame_ = nullptr;
    std::shared_ptr<Camera> camera_left_{nullptr};
    std::shared_ptr<Camera> camera_right_{nullptr};
    std::shared_ptr<Viewer> viewer_ = nullptr;

    std::vector<cv::KeyPoint> feature_grid_[NUMBER_GRID_CELL_COLS][NUMBER_GRID_CELL_ROWS];

    Status status_{INITIALIZING};
};

#endif //VISUAL_SLAM_FRONTEND_HS
