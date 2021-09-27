#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <solve/detector.h>
#include "frontend.h"

Frontend::Frontend() {}

void Frontend::pushback(cv::Mat &image_left, cv::Mat &image_right) {;

    context_.pushback(image_left, image_right);

    switch (status_) {
        case INITIALIZING:
            initialize();
            break;
        case TRACKING:
            process();
            break;
        case LOST:
            restart();
            break;
    }
}

int Frontend::initialize() {
    detector_.detect(context_);
    matcher_->match_stereo(context_);
    context_.frame_current_->setIsKeyframe(true);
    context_.viewer_ = viewer_;
    map_->insertKeyframe(context_.frame_current_);
    status_ = TRACKING;
}

int Frontend::process() {

    if ( ! context_.frame_previous_ || ! context_.frame_current_ ) return -1;

    // -----------------------------------------------------------------------------------------------------------------
    // Matching : circular match 2D features using a circular method
    // -----------------------------------------------------------------------------------------------------------------
    matcher_->match_quadro(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Triangulator : 3D map points triangulated from new stereo matched 2D features
    // -----------------------------------------------------------------------------------------------------------------
    triangulator_->triangulate(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Tracking : track features from previous frame using KLT method
    // -----------------------------------------------------------------------------------------------------------------
    tracker_->track(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Estimation : current pose estimate (PnP method) based on previous landmarks
    // -----------------------------------------------------------------------------------------------------------------
    estimation_->estimate(context_, camera_left_->K());

    // -----------------------------------------------------------------------------------------------------------------
    // Landmarks : transform triangulated features (camara frame) into world frame ready for next iteration
    // -----------------------------------------------------------------------------------------------------------------
    for ( auto &feature : context_.frame_current_->features_left_ )
    {
        auto & p3d = feature->getLandmark()->getPoint3D();
        Eigen::Vector3d v3d(p3d.x, p3d.y, p3d.z);
        v3d = context_.frame_current_->getPose() * v3d;
        p3d.x = v3d[0];
        p3d.y = v3d[1];
        p3d.z = v3d[2];
        context_.frame_current_->getLandmarks().push_back(feature->getLandmark());
    }

    if ( 0 != context_.frame_current_->getID() && context_.frame_current_->getID() % 5 == 0) {
        insertKeyframe(context_.frame_current_);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Visualization : pushback visualization with current estimated state
    // -----------------------------------------------------------------------------------------------------------------
    viewer_->displayTrajectory(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Detection : detect new features with FAST and bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if ( context_.frame_current_->getID() % 5 == 0 ) {
        backend_->updateMap();
        detector_.detect(context_);
    }
}

int Frontend::restart() {

}


void Frontend::insertKeyframe(std::shared_ptr<Frame> frame) {

    if ( ! frame->getIsKeyframe() ) {
        return;
    }

    std::cout << "[INFO] Frontend::insert_frame {" << frame->getID() << "} - input features " << frame->features_left_.size() << std::endl;
    map_->insertKeyframe(frame);
}