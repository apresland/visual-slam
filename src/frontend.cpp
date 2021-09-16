#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include "observation.h"
#include "feature.h"
#include "mappoint.h"
#include "frontend.h"

Frontend::Frontend() {}

void Frontend::update(std::shared_ptr<Frame> frame) {;

    frame->id_ = frame_id_;
    frame_next_ = frame;

    switch (status_) {
        case INITIALIZING:
            initialize(frame_next_);
            break;
        case TRACKING:
            process(frame_previous_, frame_current_, frame_next_);
            break;
        case LOST:
            restart();
            break;
    }

    frame_previous_ = frame_current_;
    frame_current_ = frame_next_;

    ++frame_id_;
}

int Frontend::initialize(std::shared_ptr<Frame> frame) {
    viewer_->load_poses();
    detector_.detect(frame, nullptr);
    matcher_->match(frame);
    triangulate(frame);
    frame->is_keyframe_ = true;
    map_->insert_keyframe(frame);
    status_ = TRACKING;
}

int Frontend::process(std::shared_ptr<Frame> frame_previous, std::shared_ptr<Frame> frame_current, std::shared_ptr<Frame> frame_next) {

    if ( ! frame_previous || ! frame_current || ! frame_next) return -1;

    // -----------------------------------------------------------------------------------------------------------------
    // Tracking : track features from previous frame using KLT method
    // -----------------------------------------------------------------------------------------------------------------
    matcher_->track(frame_previous, frame_current);

    // -----------------------------------------------------------------------------------------------------------------
    // Visualization : update visualization with current estimated state
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->update(frame_previous, frame_current);

    // -----------------------------------------------------------------------------------------------------------------
    // Estimation : estimate current pose with PnP method
    // -----------------------------------------------------------------------------------------------------------------
    estimate_pose(frame_previous, frame_current, camera_left_->K());
    if(0 != frame_current->id_) {
        insert_keyframe(frame_current);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Detection : detect new features with FAST and bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if (frame_id_ % 5 == 0) {
        detector_.detect(frame_current, frame_previous);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Matching : stereo match features using a circular method
    // -----------------------------------------------------------------------------------------------------------------
    matcher_->match(frame_current, frame_next);

    // -----------------------------------------------------------------------------------------------------------------
    // Triangulation : triangulate 3D map points from new stereo matched 2D features
    // -----------------------------------------------------------------------------------------------------------------
    triangulate(frame_current);
}

int Frontend::restart() {

}

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera get_pose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
void Frontend::estimate_pose(std::shared_ptr<Frame> frame_previous,
                             std::shared_ptr<Frame> frame_current,
                             const cv::Mat K)
{
    std::vector<cv::Point2f> points_2d = frame_current->get_points_left();
    std::cout << "[INFO] Frontend::estimate_pose - points { 2D " << points_2d.size() << " }" << std::endl;
    std::vector<cv::Point3f> points_3d = frame_current->get_points_3d();
    std::cout << "[INFO] Frontend::estimate_pose - points { 3D " << points_3d.size() << " }" << std::endl;

    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat r = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac(points_3d, points_2d, K, coeffs, r, t,
                       false, 100, 1.0, 0.999, inliers );
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

    remove_outliers(frame_current, inliers);

    double distance = cv::norm(t);
    double angle = cv::norm(R);
    std::cout << "[INFO] Frontend::esimate_pose - relative motion = " << distance << " angle = " << angle << std::endl;

    bool is_keyframe = true;
    Sophus::SE3d T_c_w = frame_previous->get_pose();
    if (distance > 0.05 && distance < 5) {
        frame_current->is_keyframe_ = true;
        T_c_w = T_c_w * T.inverse();
    } else {
        frame_current->is_keyframe_ = false;
        std::cout << "[WARNING] get_pose not updated due to out-of-bounds scale value" << distance << std::endl;
    }

    frame_current->set_pose(T_c_w);
}

void Frontend::triangulate(std::shared_ptr<Frame> frame) {

    std::vector<cv::Point2f>  matches_2d_left = frame->get_points_left();
    std::vector<cv::Point2f>  matches_2d_right = frame->get_points_right();

    std::cout << "[INFO] Frontend::triangulate - input points 2D { L"
        << matches_2d_left.size() << " : R"
        << matches_2d_right.size() << "}" << std::endl;

    cv::Mat proj_matrix_left = camera_left_->P();
    cv::Mat proj_matrix_right = camera_right_->P();
    cv::Mat points3D, points4D;
    cv::triangulatePoints(proj_matrix_left, proj_matrix_right, matches_2d_left, matches_2d_right, points4D);
    cv::convertPointsFromHomogeneous(points4D.t(), points3D);

    assert(points3D.rows == frame->features_left_.size());

    int num_predefined = 0;
    std::cout << "[INFO] Frontend::triangulate - defining " << points3D.rows << " mappoints" << std::endl;
    for(int i=0; i< points3D.rows; ++i) {
        if ( frame->features_left_[i]->landmark_ ) num_predefined++;
        cv::Point3f p3d = *points3D.ptr<cv::Point3f>(i);
        std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>();
        map_point->point_3d_ = p3d;
        frame->features_left_[i]->landmark_ = map_point;
        frame->features_right_[i]->landmark_ = map_point;
        map_->insert_landmark(*map_point);
    }

    std::cout << "[INFO] Frontend::triangulate - " << num_predefined << " predefined mappoints "
    << frame->features_left_.size() - num_predefined << " new" << std::endl;
}

void Frontend::remove_outliers(std::shared_ptr<Frame> frame, cv::Mat inliers) {

    for (int idx = 0; idx < inliers.rows; idx++)
    {
        int index = inliers.at<int>(idx, 0);
        frame->features_left_[index]->is_inlier_ = true;
    }

    std::vector<std::shared_ptr<Feature>> features_left(frame->features_left_);
    frame->features_left_.clear();
    for ( int i =0; i < features_left.size(); i++)
    {
        if ( features_left[i]->is_inlier_) frame->features_left_.push_back(features_left[i]);
    }
}

void Frontend::insert_keyframe(std::shared_ptr<Frame> frame) {

    if ( ! frame->is_keyframe_) {
        return;
    }

    std::cout << "[INFO] Frontend::insert_frame {" << frame->id_ << "} - input features " << frame->features_left_.size() << std::endl;
    map_->insert_keyframe(frame);
}