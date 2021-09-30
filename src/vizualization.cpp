#include <opencv2/opencv.hpp>
#include "vizualization.h"

void Vizualization::prepareKeypointsVisual(const Context &context, cv::Mat& output_image) {

    if ( ! context.frame_current_) {
        return  ;
    }

    const cv::Mat &input_image = context.frame_current_->image_left_;
    output_image = input_image.clone();

    if( ! context.keypoints_.empty()) {
        std::vector<cv::KeyPoint> keypoints = context.keypoints_;
        cv::drawKeypoints(input_image, keypoints, output_image,
                          CV_RGB(0, 128, 255),
                          cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    }
}

void Vizualization::prepareFeaturesVisual(const Context &context, cv::Mat& output_image) {

    cv::Mat &input_image = context.frame_current_->image_left_;
    output_image = input_image.clone();

    if ( ! context.frame_current_) {
        return  ;
    }

    if( ! context.frame_current_->getPointsLeft().empty() ) {
        std::vector<cv::KeyPoint> keypoints;
        cv::KeyPoint::convert(context.frame_current_->getPointsLeft(), keypoints, 5, 5, 0, -1);
        cv::drawKeypoints(input_image, keypoints, output_image, CV_RGB(0, 255, 0));
    }
}

void Vizualization::prepareOpticalFlowVizual(const Context &context, cv::Mat& output_image) {

    const cv::Mat &input_image = context.frame_current_->image_left_;
    cv::cvtColor(input_image, output_image, cv::COLOR_GRAY2BGR);

    if ( ! context.frame_current_) {;
        return;
    }

    assert(context.flow_points_2d_t0.size() == context.flow_points_2d_t1.size());

    for (int i = 0; i < context.flow_points_2d_t0.size(); i++) {
        cv::circle(output_image, context.flow_points_2d_t0[i], 2, CV_RGB(0, 255, 0));
        cv::circle(output_image, context.flow_points_2d_t1[i], 2, CV_RGB(255, 0, 0));
        cv::line(output_image, context.flow_points_2d_t0[i], context.flow_points_2d_t1[i], CV_RGB(0, 255, 0));
    }
}

