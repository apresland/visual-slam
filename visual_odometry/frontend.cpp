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
    // Estimate Pose :
    // -----------------------------------------------------------------------------------------------------------------
    estimate_pose_3d2d_ransac(points_left_t1, points3D_t0, camera_left_->K(), T_c_w_);
    features_ = points_left_t1;

    // -----------------------------------------------------------------------------------------------------------------
    // Visualize Results :
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->display_features(image_left_t1, points_left_t1);
    viewer_->display_tracking(image_left_t1, points_left_t0, points_left_t1);
    viewer_->display_trajectory(T_c_w_, frame_id_);
}

int Frontend::restart() {

}

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera pose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
int Frontend::estimate_pose_3d2d_ransac(const std::vector<cv::Point2f>&  image_points_2d,
                                        const cv::Mat& object_points_3d,
                                        const cv::Mat K,
                                        Sophus::SE3d &T_c_w)
{
    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat r = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac( object_points_3d, image_points_2d, K, coeffs, r, t,
                        false, 100, 8.0, 0.99,inliers );
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Rodrigues(r, R);

    // Relative motion (T)
    Eigen::Matrix3d SO3_R;
    Eigen::Vector3d SO3_t;
    SO3_R << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    SO3_t << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);
    Sophus::SE3d T = Sophus::SE3d(SO3_R, SO3_t);

    // Scale of motion check
    double motion_scale = sqrt(T.translation().x() * T.translation().x()
                        + T.translation().y() * T.translation().y()
                        + T.translation().z() * T.translation().z());
    if (motion_scale > 0.05 && motion_scale < 10) {
        T_c_w = T_c_w * T.inverse();
    } else {
        std::cout << "[WARNING] Pose not updated due to out-of-bounds scale value" << motion_scale << std::endl;
    }

    return inliers.rows;
}