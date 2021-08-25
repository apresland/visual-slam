#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include "feature.h"
#include "frontend.h"

Frontend::Frontend() {
    detector_ = cv::FastFeatureDetector::create(20, true);
}

void Frontend::process(std::shared_ptr<Frame> frame) {

    current_frame_ = frame;

    switch (status_) {
        case INITIALIZING:
            initialize();
            break;
        case TRACKING:
            track();
            break;
        case LOST:
            restart();
            break;
    }

    if(viewer_) {
        viewer_->view(previous_frame_, current_frame_, current_features_);
    }

    previous_frame_ = current_frame_;
}

int Frontend::initialize() {
    detect_features(current_frame_->image_left_, current_features_);
    status_ = TRACKING;
}

int Frontend::track() {

    // input images
    cv::Mat &imgL_t0 = previous_frame_->image_left_;
    cv::Mat &imgR_t0 = previous_frame_->image_right_;
    cv::Mat &imgL_t1 = current_frame_->image_left_;
    cv::Mat &imgR_t1 = current_frame_->image_right_;

    // circular feature matching outputs
    std::vector<cv::Point2f> ptsL_t0, ptsR_t0, ptsL_t1, ptsR_t1;
    std::vector<cv::Point2f>  matched_features;

    // create more features when count low
    if (current_features_.size() < MIN_FEATURE_COUNT) {
        detect_features(imgL_t0, current_features_);
    }

    // stereo match features
    ptsL_t0 = current_features_;
    match_features(imgL_t0, imgR_t0,imgL_t1, imgR_t1,
                        ptsL_t0, ptsR_t0,ptsL_t1, ptsR_t1,
                        matched_features);

    current_features_ = ptsL_t1;
}

int Frontend::restart() {

}

int Frontend::detect_features(cv::Mat &image, std::vector<cv::Point2f> &current_features) {

    // detect features with FAST
    std::vector<cv::KeyPoint> keypoints;
    detector_->detect(image, keypoints);
    std::vector<cv::Point2f> points;
    cv::KeyPoint::convert(keypoints, points);
    current_features.insert(current_features.end(), points.begin(), points.end());
    for (auto &keypoint : keypoints) {
        current_frame_->keypoints_left_.push_back(keypoint);
    }

    auto sort_predicate = [](cv::KeyPoint const& kp1, cv::KeyPoint const& kp2)-> bool {
        return kp1.response > kp2.response;
    };

    // clear feature grid before repopulating
    for(int x =0; x < NUMBER_GRID_CELL_COLS; ++x) {
        for(int y =0; y < NUMBER_GRID_CELL_ROWS; ++y) {
            feature_grid_[x][y].clear();
        }
    }

    // populate the grid with current features
    for (auto &point : current_features) {
        int x, y;
        if(position_in_grid(point, x, y))
            feature_grid_[x][y].push_back(point);
    }

    // flatten grid with filtering
    current_features.clear();
    for(int x =0; x < NUMBER_GRID_CELL_COLS; ++x) {
        for(int y =0; y < NUMBER_GRID_CELL_ROWS; ++y) {
            unsigned int count = 0;
            //std::sort(feature_grid_[x][y].begin(), feature_grid_[x][y].end(), sort_predicate);
            for (auto &point : feature_grid_[x][y]) {
                if (count >= MAX_FEATURES_PER_CELL) continue;
                std::shared_ptr<Feature> feature = std::make_shared<Feature>(point);
                current_frame_->features_left_.push_back(feature);
                current_features.push_back(point);
                ++count;
            }
        }
    }
}

int Frontend::match_features(
        cv::Mat imgL_t0, cv::Mat imgR_t0, cv::Mat imgL_t1, cv::Mat imgR_t1,
        std::vector<cv::Point2f> &ptsL_t0, std::vector<cv::Point2f> &ptsR_t0,
        std::vector<cv::Point2f> &ptsL_t1, std::vector<cv::Point2f> &ptsR_t1,
        std::vector<cv::Point2f> &ptsL_t0_return){

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    std::vector<uchar> status_0;
    std::vector<uchar> status_1;
    std::vector<uchar> status_2;
    std::vector<uchar> status_3;

    // circular matching with LK optical flow
    calcOpticalFlowPyrLK(imgL_t0, imgL_t1, ptsL_t0, ptsL_t1, status_0, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(imgL_t1, imgR_t1, ptsL_t1, ptsR_t1, status_1, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(imgR_t1, imgR_t0, ptsR_t1, ptsR_t0, status_2, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(imgR_t0, imgL_t0, ptsR_t0, ptsL_t0_return, status_3, err, window_size, 3, term_criteria, 0, 0.001);

    // filter out bad matches
    int deletion_correction = 0;
    for(int i=0; i < status_3.size(); i++)
    {
        // get next points respecting prior deletions
        cv::Point2f point_0 = ptsL_t0.at(i - deletion_correction);
        cv::Point2f point_1 = ptsR_t0.at(i - deletion_correction);
        cv::Point2f point_2 = ptsL_t1.at(i - deletion_correction);
        cv::Point2f point_3 = ptsR_t1.at(i - deletion_correction);

        // check for bad status and out-of-frame points
        if ((status_3.at(i) == 0) || (point_3.x < 0) || (point_3.y < 0) ||
            (status_2.at(i) == 0) || (point_2.x < 0) || (point_2.y < 0) ||
            (status_1.at(i) == 0) || (point_1.x < 0) || (point_1.y < 0) ||
            (status_0.at(i) == 0) || (point_0.x < 0) || (point_0.y < 0))
        {
            // erase bad matches
            ptsL_t0.erase (ptsL_t0.begin() + (i - deletion_correction));
            ptsR_t0.erase (ptsR_t0.begin() + (i - deletion_correction));
            ptsL_t1.erase (ptsL_t1.begin() + (i - deletion_correction));
            ptsR_t1.erase (ptsR_t1.begin() + (i - deletion_correction));
            ptsL_t0_return.erase (ptsL_t0_return.begin() + (i - deletion_correction));

            deletion_correction++;
        }
    }
}

bool Frontend::position_in_grid(cv::Point2f &point, int &grid_pos_x, int &grid_pos_y) {
    float grid_cell_width_inverse = static_cast<float>(NUMBER_GRID_CELL_COLS) / (current_frame_->image_left_.cols);
    float grid_cell_height_inverse = static_cast<float>(NUMBER_GRID_CELL_ROWS) / (current_frame_->image_left_.rows);
    grid_pos_x = round((point.x)*grid_cell_width_inverse);
    grid_pos_y = round((point.y)*grid_cell_height_inverse);
    if(grid_pos_x<0 || grid_pos_x >= NUMBER_GRID_CELL_COLS || grid_pos_y < 0 || grid_pos_y >= NUMBER_GRID_CELL_ROWS)
        return false;

    return true;
}

