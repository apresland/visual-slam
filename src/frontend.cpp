#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include "observation.h"
#include "feature.h"
#include "landmark.h"
#include "frontend.h"

Frontend::Frontend() {}

void Frontend::update(std::shared_ptr<Frame> frame) {;

    frame_t1_ = frame;

    switch (status_) {
        case INITIALIZING:
            initialize(frame_t1_);
            break;
        case TRACKING:
            process(frame_t0_, frame_t1_);
            break;
        case LOST:
            restart();
            break;
    }

    frame_t0_ = frame_t1_;
    ++frame_id_;
}

int Frontend::initialize(std::shared_ptr<Frame> frame) {
    frame_id_ = 0;
    viewer_->load_poses();
    detector_.detect(frame);
    status_ = TRACKING;
}

int Frontend::process(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {

    // -----------------------------------------------------------------------------------------------------------------
    // Detect features : FAST keypoint detection with grid-based bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if (frame_t0->features_left_.size() < MIN_FEATURE_COUNT) {
        detector_.detect(frame_t0);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Trackable features : Select features seen at t0 and t1 with Lucas-Kanade tracking and circular matching.
    // -----------------------------------------------------------------------------------------------------------------
    tracker_.track(frame_t0, frame_t1);

    // -----------------------------------------------------------------------------------------------------------------
    // Triangulate features : Get 3D positions of tracked features at t0 w.r.t. camera
    // -----------------------------------------------------------------------------------------------------------------
    triangulate(frame_t0);

    // -----------------------------------------------------------------------------------------------------------------
    // Estimate pose : Camera get_pose in world coordinates at time t1 by reprojecting features from t0
    // -----------------------------------------------------------------------------------------------------------------
    bool is_keyframe = estimate_pose(frame_t0, frame_t1, camera_left_->K());


    // -----------------------------------------------------------------------------------------------------------------
    //
    // -----------------------------------------------------------------------------------------------------------------
    if(is_keyframe) { insert_keyframe(frame_t1); }


    // -----------------------------------------------------------------------------------------------------------------
    // Visualize Results :
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->display_features(frame_t1);
    viewer_->display_tracking(frame_t0, frame_t1);
    viewer_->display_trajectory(frame_t1->get_pose(), frame_id_);
}

int Frontend::restart() {

}

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera get_pose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
bool Frontend::estimate_pose(std::shared_ptr<Frame> frame_t0,
                             std::shared_ptr<Frame> frame_t1,
                             const cv::Mat K)
{
    std::vector<cv::Point3f>& object_points_3d = frame_t0->points_3d_;
    std::vector<cv::Point2f> image_points_2d;
    for( auto & f : frame_t1_->features_left_) {
        image_points_2d.push_back(f->point_2d_);
    }

    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat r = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac( object_points_3d, image_points_2d, K, coeffs, r, t,
                        false, 100, 2.0, 0.999,inliers );
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

    remove_outliers(frame_t0, inliers);
    remove_outliers(frame_t1, inliers);

    double distance = cv::norm(t);
    double angle = cv::norm(R);
    std::cout << "[INFO] Relative motion = " << distance << " angle = " << angle << std::endl;

    bool is_keyframe = true;
    Sophus::SE3d T_c_w = frame_t0->get_pose();
    if (distance > 0.05 && distance < 5) {
        T_c_w = T_c_w * T.inverse();
    } else {
        is_keyframe = false;
        std::cout << "[WARNING] get_pose not updated due to out-of-bounds scale value" << distance << std::endl;
    }

    frame_t1->set_pose(T_c_w);

    return is_keyframe;
}

void Frontend::triangulate(std::shared_ptr<Frame> frame_t0) {

    std::vector<cv::Point2f> frame_t0_points_left_ = frame_t0->get_points_left();
    std::vector<cv::Point2f> frame_t0_points_right_ = frame_t0->get_points_right() ;

    cv::Mat proj_matrix_left = camera_left_->P();
    cv::Mat proj_matrix_right = camera_right_->P();
    cv::Mat points3D, points4D;
    cv::triangulatePoints( proj_matrix_left,  proj_matrix_right,  frame_t0_points_left_,  frame_t0_points_right_,  points4D);
    cv::convertPointsFromHomogeneous(points4D.t(), points3D);
    for(int i=0; i< points3D.rows; ++i) {
        cv::Point3f p = *points3D.ptr<cv::Point3f>(i);
        frame_t0->points_3d_.push_back(p);
        Eigen::Matrix<double, 3, 1> pworld(p.x, p.y, p.z);
        pworld = T_c_w_ * pworld;
        std::shared_ptr<Landmark> landmark = std::make_shared<Landmark>();
        landmark->set_position(pworld);
        landmark->add_observation(frame_t0->features_left_[i]);
        landmark->add_observation(frame_t0->features_right_[i]);
        frame_t0->features_left_[i]->landmark_ = landmark;
        frame_t0->features_right_[i]->landmark_ = landmark;
        map_->insert_landmark(landmark);
    }
}

void Frontend::insert_keyframe(std::shared_ptr<Frame> frame_t1) {

    for (auto &feature : frame_t1->features_left_) {
        auto landmark = feature->landmark_.lock();
        if (landmark) landmark->add_observation(feature);
    }

    map_->insert_keyframe(frame_t1);
}

void Frontend::remove_outliers(std::shared_ptr<Frame> frame, cv::Mat inliers) {

    for (int idx = 0; idx < inliers.rows; idx++) {
        int index = inliers.at<int>(idx, 0);
        frame->features_left_.at(index)->is_inlier_ = true;
    }

    frame->features_left_.erase(
            std::remove_if(
                    frame->features_left_.begin(),
                    frame->features_left_.end(),
                    [](const std::shared_ptr<Feature> &x) {
                        return !x->is_inlier_;
                    }),
            frame->features_left_.end());
}