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
    std::cout << "Frame:" << frame->id_ << std::endl;

    frame_current_ = frame;

    switch (status_) {
        case INITIALIZING:
            initialize(frame_current_);
            break;
        case TRACKING:
            process(frame_previous_, frame_current_);
            break;
        case LOST:
            restart();
            break;
    }

    frame_previous_ = frame_current_;

    ++frame_id_;
}

int Frontend::initialize(std::shared_ptr<Frame> frame) {
    viewer_->load_poses();
    detector_.detect(frame, nullptr);
    matcher_.match(frame);
    triangulate(frame);
    /**
for (int i = 0; i < frame->features_left_.size(); i++) {
    std::shared_ptr<Feature> feature = frame->features_left_[i];
    feature->id_ = feature_id_;
    feature->frame_id_ = frame->id_;
    feature->landmark_id_ = landmark_id_;
    Observation observation(frame->id_, feature->id_);
    MapPoint landmark(landmark_id_, frame->features_left_[i]->landmark_->point_3d_);
    landmark.add_observation(observation);
    map_->insert_landmark(landmark);
    landmark_id_++;
    feature_id_++;
}
*/
    frame->is_keyframe_ = true;
    //map_->insert_keyframe(frame);
    status_ = TRACKING;
}

int Frontend::process(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {

    // -----------------------------------------------------------------------------------------------------------------
    // Estimation :
    // -----------------------------------------------------------------------------------------------------------------
    matcher_.match(frame_t0);
    triangulate(frame_t0);

    matcher_.match_circular(frame_t0, frame_t1);
    estimate_pose(frame_t0, frame_t1, camera_left_->K());

    if(0 != frame_t0->id_) {
        //insert_keyframe(frame_t1);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Visualization :
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->display_features(frame_t1);
    viewer_->display_tracking(frame_t0, frame_t1);
    viewer_->display_trajectory(frame_t1->get_pose(), frame_id_);


    // -----------------------------------------------------------------------------------------------------------------
    // Detection :
    // -----------------------------------------------------------------------------------------------------------------
    if (frame_id_ % 5 == 0) {
        detector_.detect(frame_t1, frame_t0);
    }
}

int Frontend::restart() {

}

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera get_pose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
void Frontend::estimate_pose(std::shared_ptr<Frame> frame_t0,
                             std::shared_ptr<Frame> frame_t1,
                             const cv::Mat K)
{
    std::vector<cv::Point2f> points_2d_t1 = frame_t1->get_points_left();
    std::cout << "[INFO] Frontend::estimate_pose - points { 2D " << points_2d_t1.size() << " }" << std::endl;
    std::vector<cv::Point3f> points_3d_t0 = frame_t0->get_points_3d();
    std::cout << "[INFO] Frontend::estimate_pose - points { 3D " << points_3d_t0.size() << " }" << std::endl;

    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat r = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac(points_3d_t0, points_2d_t1, K, coeffs, r, t,
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

    remove_outliers(frame_t0, inliers);
    remove_outliers(frame_t1, inliers);

    double distance = cv::norm(t);
    double angle = cv::norm(R);
    std::cout << "[INFO] Frontend::esimate_pose - relative motion = " << distance << " angle = " << angle << std::endl;

    bool is_keyframe = true;
    Sophus::SE3d T_c_w = frame_t0->get_pose();
    if (distance > 0.05 && distance < 5) {
        frame_t1->is_keyframe_ = true;
        T_c_w = T_c_w * T.inverse();
    } else {
        frame_t1->is_keyframe_ = false;
        std::cout << "[WARNING] get_pose not updated due to out-of-bounds scale value" << distance << std::endl;
    }

    frame_t1->set_pose(T_c_w);
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

    for(int i=0; i< points3D.rows; ++i) {
        cv::Point3f p3d = *points3D.ptr<cv::Point3f>(i);
        std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>();
        map_point->point_3d_ = p3d;
        frame->features_left_[i]->landmark_ = map_point;
        frame->features_right_[i]->landmark_ = map_point;
    }

    std::cout << "[INFO] Frontend::triangulate - output points 3D { " << points3D.rows << " }" << std::endl;
}

void Frontend::remove_outliers(std::shared_ptr<Frame> frame, cv::Mat inliers) {

    for (int idx = 0; idx < inliers.rows; idx++) {
        int index = inliers.at<int>(idx, 0);
        frame->features_left_[index]->is_inlier_ = true;
        frame->features_right_[index]->is_inlier_ = true;
    }

    std::vector<std::shared_ptr<Feature>> features_left(frame->features_left_);
    frame->features_left_.clear();
    for ( int i =0; i < features_left.size(); i++)
        if ( features_left[i]->is_inlier_) frame->features_left_.push_back(features_left[i]);

    std::vector<std::shared_ptr<Feature>> features_right(frame->features_right_);
    frame->features_right_.clear();
    for ( int i =0; i < features_right.size(); i++)
        if ( features_right[i]->is_inlier_) frame->features_right_.push_back(features_right[i]);
}

void Frontend::insert_keyframe(std::shared_ptr<Frame> frame) {

    if ( ! frame->is_keyframe_) {
        return;
    }

    std::cout << "[INFO] Frontend::insert_frame {" << frame->id_ << "} - input features " << frame->features_left_.size() << std::endl;

    // Repeat observation of existing landmark
    int numb_repeat_observations = 0;
    for (int i = 0; i < frame->features_left_.size(); ++i) {
        int frame_id = frame->id_;
        int feature_id = frame->features_left_.at(i)->id_;
        unsigned long landmark_id = frame->features_left_.at(i)->landmark_id_;
        Observation observation(frame_id, feature_id);
        //map_->landmarks_.at(landmark_id).add_observation(observation);
        ++numb_repeat_observations;
    }

    map_->insert_keyframe(frame);
}