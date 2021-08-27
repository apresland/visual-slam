#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
#include "frontend.h"

Frontend::Frontend() {}

void Frontend::update(const cv::Mat &image_left_t1, const cv::Mat &image_right_t1) {

    switch (status_) {
        case INITIALIZING:
            initialize(image_left_t1);
            break;
        case TRACKING:
            process(image_left_t0, image_right_t0, image_left_t1, image_right_t1);
            break;
        case LOST:
            restart();
            break;
    }

    image_left_t1.copyTo(image_left_t0);
    image_right_t1.copyTo(image_right_t0);
}

int Frontend::initialize(const cv::Mat &image_left_t1) {
    frame_id_ = 0;
    viewer_->load_poses();
    detector_.detect(image_left_t1, features_);
    status_ = TRACKING;
}

int Frontend::process(const cv::Mat &image_left_t0, const cv::Mat &image_right_t0,
                      const cv::Mat &image_left_t1, const cv::Mat &image_right_t1) {

    ++frame_id_;

    std::vector<cv::Point2f>  matched_features;
    std::vector<cv::Point2f> points_left_t0, points_right_t0, points_left_t1, points_right_t1;

    // -----------------------------------------------------------------------------------------------------------------
    // Detect features : FAST keypoint detection with grid-based bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if (features_.size() < MIN_FEATURE_COUNT) {
        detector_.detect(image_left_t0, features_);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Track Features : Lucas-Kanade tracking with circular matching.
    // -----------------------------------------------------------------------------------------------------------------
    points_left_t0 = features_;
    tracker_.track(image_left_t0, image_right_t0, image_left_t1, image_right_t1,
                   points_left_t0, points_right_t0, points_left_t1, points_right_t1,
                   matched_features);

    // -----------------------------------------------------------------------------------------------------------------
    // Triangulate 3D Points :
    // -----------------------------------------------------------------------------------------------------------------
    cv::Mat proj_matrix_left = camera_left_->P();
    cv::Mat proj_matrix_right = camera_right_->P();
    cv::Mat points3D_t0, points4D_t0;
    cv::triangulatePoints( proj_matrix_left,  proj_matrix_right,  points_left_t0,  points_right_t0,  points4D_t0);
    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

    // -----------------------------------------------------------------------------------------------------------------
    // Estimate Motion :
    // -----------------------------------------------------------------------------------------------------------------
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    estimate_motion(points_left_t1, points3D_t0, camera_left_->K(), R, t);
    features_ = points_left_t1;

    // -----------------------------------------------------------------------------------------------------------------
    // Update Pose :
    // -----------------------------------------------------------------------------------------------------------------
    update_pose(pose_, R, t);

    // -----------------------------------------------------------------------------------------------------------------
    // Visualize Results :
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->display_features(image_left_t1, points_left_t1);
    viewer_->display_tracking(image_left_t1, points_left_t0, points_left_t1);
    viewer_->display_trajectory(pose_, frame_id_);
}

int Frontend::restart() {

}

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera pose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
void Frontend::estimate_motion(const std::vector<cv::Point2f>&  image_points_2d,
                               const cv::Mat& object_points_3d,
                               const cv::Mat K,
                               const cv::Mat& R,
                               const cv::Mat& t)
{
    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat r = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac( object_points_3d, image_points_2d, K, coeffs, r, t,
                        false, 100, 8.0, 0.99,inliers );
    cv::Rodrigues(r, R);
    std::cout << "Inliers: " << inliers << std::endl;
}

void Frontend::update_pose(cv::Mat& pose, const cv::Mat& R, const cv::Mat& t)
{
    cv::Mat T_c_w;
    cv::Mat sum = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);
    cv::hconcat(R, t, T_c_w);
    cv::vconcat(T_c_w, sum, T_c_w);
    T_c_w = T_c_w.inv();

    double scale = sqrt((t.at<double>(0)) * (t.at<double>(0))
                        + (t.at<double>(1)) * (t.at<double>(1))
                        + (t.at<double>(2)) * (t.at<double>(2))) ;
    if (scale > 0.05 && scale < 10) {
        pose = pose * T_c_w;
    } else {
        std::cout << "[WARNING] Pose not updated due to out-of-bounds scale value" << scale << std::endl;
    }
}
