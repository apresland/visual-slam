#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <solve/detector.h>
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
    viewer_->loadGroundTruthPoses();
    detector_.detect(frame, nullptr);
    matcher_->match(frame);
    triangulator_->triangulate(frame);
    frame->is_keyframe_ = true;
    map_->insertKeyframe(frame);
    status_ = TRACKING;
}

int Frontend::process(std::shared_ptr<Frame> frame_previous, std::shared_ptr<Frame> frame_current, std::shared_ptr<Frame> frame_next) {

    if ( ! frame_previous || ! frame_current || ! frame_next) return -1;

    // -----------------------------------------------------------------------------------------------------------------
    // Tracking : track features from previous frame using KLT method
    // -----------------------------------------------------------------------------------------------------------------
    tracker_->track(frame_previous, frame_current);

    // -----------------------------------------------------------------------------------------------------------------
    // Estimation : current pose estimate (PnP method) based on previous landmarks
    // -----------------------------------------------------------------------------------------------------------------
    estimation_->estimate(frame_previous, frame_current, camera_left_->K());

    // -----------------------------------------------------------------------------------------------------------------
    // Landmarks : transform triangulated features (camara frame) into world frame ready for next iteration
    // -----------------------------------------------------------------------------------------------------------------
    for ( auto &feature : frame_current->features_left_ )
    {
        auto & p3d = feature->landmark_->point_3d_;
        Eigen::Vector3d v3d(p3d.x, p3d.y, p3d.z);
        v3d = frame_current->getPose() * v3d;
        feature->landmark_->point_3d_.x = v3d[0];
        feature->landmark_->point_3d_.y = v3d[1];
        feature->landmark_->point_3d_.z = v3d[2];
        frame_current->landmarks_.push_back(feature->landmark_);
    }

    if ( 0 != frame_current->id_ && frame_current->id_ % 5 == 0) {
        insertKeyframe(frame_current);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Visualization : update visualization with current estimated state
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->update(frame_previous, frame_current);

    // -----------------------------------------------------------------------------------------------------------------
    // Detection : detect new features with FAST and bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if ( frame_id_ % 5 == 0 ) {
        backend_->updateMap();
        detector_.detect(frame_current, frame_previous);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Matching : stereo match 2D features using a circular method
    // -----------------------------------------------------------------------------------------------------------------
    matcher_->match(frame_current, frame_next);

    // -----------------------------------------------------------------------------------------------------------------
    // Triangulator : 3D map points triangulated from new stereo matched 2D features
    // -----------------------------------------------------------------------------------------------------------------
    triangulator_->triangulate(frame_current);
}

int Frontend::restart() {

}


void Frontend::insertKeyframe(std::shared_ptr<Frame> frame) {

    if ( ! frame->is_keyframe_) {
        return;
    }

    std::cout << "[INFO] Frontend::insert_frame {" << frame->id_ << "} - input features " << frame->features_left_.size() << std::endl;
    map_->insertKeyframe(frame);
}