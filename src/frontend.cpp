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
    triangulator_->triangulate(frame);
    for (auto &f : frame->features_left_)
    {
        auto & p3d = f->landmark_->point_3d_;
        Eigen::Vector3d v3d(p3d.x, p3d.y, p3d.z);
        v3d = frame->get_pose() * v3d;
        p3d.x = v3d[0];
        p3d.y = v3d[1];
        p3d.z = v3d[2];
    }
    frame->is_keyframe_ = true;
    map_->insert_keyframe(frame);
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
    for (auto &f : frame_current->features_left_)
    {
        auto & p3d = f->landmark_->point_3d_;
        Eigen::Vector3d v3d(p3d.x, p3d.y, p3d.z);
        v3d = frame_previous->get_pose() * v3d;
        p3d.x = v3d[0];
        p3d.y = v3d[1];
        p3d.z = v3d[2];
    }

    if(0 != frame_current->id_) {
        insert_keyframe(frame_current);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Visualization : update visualization with current estimated state
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->update(frame_previous, frame_current);

    // -----------------------------------------------------------------------------------------------------------------
    // Detection : detect new features with FAST and bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if (frame_id_ % 5 == 0) {
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


void Frontend::insert_keyframe(std::shared_ptr<Frame> frame) {

    if ( ! frame->is_keyframe_) {
        return;
    }

    std::cout << "[INFO] Frontend::insert_frame {" << frame->id_ << "} - input features " << frame->features_left_.size() << std::endl;
    map_->insert_keyframe(frame);
}