#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include "observation.h"
#include "feature.h"
#include "landmark.h"
#include "frontend.h"

Frontend::Frontend() {}

void Frontend::update(std::shared_ptr<Frame> frame) {;

    std::cout << "Frame:" << frame_id_ << std::endl;

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
    detector_.detect(frame);
    status_ = TRACKING;
}

int Frontend::process(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {

    // -----------------------------------------------------------------------------------------------------------------
    // Detection :
    // -----------------------------------------------------------------------------------------------------------------
    if (frame_id_ % 5 == 0) {
        detector_.detect(frame_t0);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Estimation :
    // -----------------------------------------------------------------------------------------------------------------
    tracker_.track(frame_t0, frame_t1);
    triangulate(frame_t0);
    insert_keyframe(frame_t0);
    estimate_pose(frame_t0, frame_t1, camera_left_->K());

    // -----------------------------------------------------------------------------------------------------------------
    // Visualization :
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
void Frontend::estimate_pose(std::shared_ptr<Frame> frame_t0,
                             std::shared_ptr<Frame> frame_t1,
                             const cv::Mat K)
{
    std::vector<cv::Point3f>& object_points_3d = frame_t0->points_3d_;
    std::vector<cv::Point2f> image_points_2d;
    for( auto &f : frame_t1->features_left_) {
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

    //remove_outliers(frame_t0, inliers);
    //remove_outliers(frame_t1, inliers);

    double distance = cv::norm(t);
    double angle = cv::norm(R);
    std::cout << "[INFO] Relative motion = " << distance << " angle = " << angle << std::endl;

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

    std::vector<cv::Point2f> points_left = frame->get_points_left();
    std::vector<cv::Point2f> points_right = frame->get_points_right() ;

    cv::Mat proj_matrix_left = camera_left_->P();
    cv::Mat proj_matrix_right = camera_right_->P();
    cv::Mat points3D, points4D;
    cv::triangulatePoints(proj_matrix_left, proj_matrix_right, points_left, points_right, points4D);
    cv::convertPointsFromHomogeneous(points4D.t(), points3D);

    for(int i=0; i< points3D.rows; ++i) {
        cv::Point3f p = *points3D.ptr<cv::Point3f>(i);
        frame->points_3d_.push_back(p);
    }
}

void Frontend::insert_keyframe(std::shared_ptr<Frame> frame) {

    if ( ! frame->is_keyframe_) {
        return;
    }

    // Repeat observation of existing landmark
    for (int i = 0; i < frame->features_left_.size(); ++i) {
        int frame_id = frame->id_;
        int feature_id = frame->features_left_.at(i)->id_;
        unsigned long landmark_id = frame->features_left_.at(i)->landmark_id_;
        Observation observation(frame_id, feature_id);
        //map_->landmarks_.at(landmark_id).add_observation(observation);
    }

    // Initial observation of novel landmark
    int feature_id = frame->features_left_.size();
    for (int i = 0; i<frame->keypoints_left_.size(); i++) {

        bool is_existing_feature = false;
        for (auto &feature: frame->features_left_) {
            is_existing_feature = ( feature->point_2d_.x == frame->keypoints_left_.at(i).pt.x
                                    && feature->point_2d_.y == frame->keypoints_left_.at(i).pt.y);
            if (is_existing_feature) {
                map_->landmarks_.at(feature->landmark_id_).point_3d_ = feature->point_3d_;
                // ToDo: Break this loop
            }
        }
        if ( ! is_existing_feature) {
            Feature feature(frame->keypoints_left_.at(i).pt);
            feature.id_ = feature_id;
            feature.frame_id_ = frame->id_;
            feature.landmark_id_ = landmark_id_;
            //frame_current_.features_.push_back(feature_to_add);
            Observation observation(frame->id_, feature.id_);
            Landmark landmark(landmark_id_, frame->points_3d_.at(i));
            map_->insert_landmark(landmark);
            landmark_id_++;
            feature_id++;
        }

    }

    map_->insert_keyframe(frame);
}

void Frontend::remove_outliers(std::shared_ptr<Frame> frame, cv::Mat inliers) {

    for (int idx = 0; idx < inliers.rows; idx++) {
        int index = inliers.at<int>(idx, 0);
        frame->features_left_.at(index)->is_inlier_ = true;
        frame->features_right_.at(index)->is_inlier_ = true;
    }

    frame->features_left_.erase(
            std::remove_if(
                    frame->features_left_.begin(),
                    frame->features_left_.end(),
                    [](const std::shared_ptr<Feature> &x) {
                        return !x->is_inlier_;
                    }),
            frame->features_left_.end());

    frame->features_right_.erase(
            std::remove_if(
                    frame->features_right_.begin(),
                    frame->features_right_.end(),
                    [](const std::shared_ptr<Feature> &x) {
                        return !x->is_inlier_;
                    }),
            frame->features_right_.end());
}