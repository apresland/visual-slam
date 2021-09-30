#include <algorithm>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <solve/detector.h>
#include <optimize/optimization.h>
#include "frontend.h"

Frontend::Frontend() {
}

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

    // -----------------------------------------------------------------------------------------------------------------
    // Detection : detect new features with FAST and bucketing
    // -----------------------------------------------------------------------------------------------------------------
    detector_.detect(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Matching : stereo match 2D features using a circular method on 2 images (left/right)
    // -----------------------------------------------------------------------------------------------------------------
    matcher_->match_stereo(context_);

    context_.frame_current_->setIsKeyframe(true);
    map_->insertKeyframe(context_.frame_current_);
    status_ = TRACKING;
}

int Frontend::process() {

    if ( ! context_.frame_previous_ || ! context_.frame_current_ ) return -1;

    context_.keypoints_.clear();

    // -----------------------------------------------------------------------------------------------------------------
    // Matching : circular match 2D features on 4 images (prev/current x left/right)
    // -----------------------------------------------------------------------------------------------------------------
    matcher_->match_quadro(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Triangulator : 3D map points triangulated from new stereo matched 2D features
    // -----------------------------------------------------------------------------------------------------------------
    triangulator_->triangulate(context_, false);

    // -----------------------------------------------------------------------------------------------------------------
    // Tracking : track features from previous frame using KLT method
    // -----------------------------------------------------------------------------------------------------------------
    tracker_->track(context_);

    // -----------------------------------------------------------------------------------------------------------------
    // Estimation : current pose estimate (PnP method) based on previous landmarks
    // -----------------------------------------------------------------------------------------------------------------
    estimation_->estimate(context_, camera_left_->K());

    if ( 0 != context_.frame_current_->getID() && context_.frame_current_->getID() % 5 == 0) {
        insertKeyframe(context_.frame_previous_);
    }

    // -----------------------------------------------------------------------------------------------------------------
    // Detection : detect new features with FAST and bucketing
    // -----------------------------------------------------------------------------------------------------------------
    if ( context_.frame_current_->getID() % 5 == 0 ) {
        detector_.detect(context_);
    }

    if ( context_.frame_current_->getID() % 10 == 0 ) {
        backend_->updateMap();
    }
}

int Frontend::restart() {

}


void Frontend::insertKeyframe(std::shared_ptr<Frame> &frame) {

    if ( ! frame->getIsKeyframe() ) {
        return;
    }

    std::cout << "[INFO] Frontend::insert_frame {" << frame->getID() << "} - landmarks " << frame->landmarks_.size() << std::endl;
    map_->insertKeyframe(frame);
}