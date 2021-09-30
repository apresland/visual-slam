#include <opencv2/opencv.hpp>
#include "vizualization.h"

cv::Mat Vizualization::prepareFeaturesVisual(const Context &context) {

    const cv::Mat &input_image = context.frame_current_->image_left_;
    std::vector<cv::Point2f> points_left_t1 = context.frame_current_->getPointsLeft();
    std::vector<cv::KeyPoint> keypoints;
    for(auto point : points_left_t1) {
        keypoints.push_back(cv::KeyPoint(point, 1.f));
    }
    cv::Mat output_image;
    cv::drawKeypoints(input_image, keypoints, output_image, CV_RGB(0, 255, 0));
    return output_image;
}

cv::Mat Vizualization::prepareOpticalFlowVizual(const Context &context) {

    const cv::Mat &input_image = context.frame_current_->image_left_;
    cv::Mat output_image;

    std::vector<cv::Point2f> points_left_t0;
    std::vector<cv::Point2f> points_left_t1;
    for(int i=0; i < context.frame_current_->features_left_.size(); i++) {
        points_left_t0.push_back(context.frame_previous_->features_left_[i]->getPoint2D());
        points_left_t1.push_back(context.frame_current_->features_left_[i]->getPoint2D());
    }

    int radius = 2;

    for (int i = 0; i < points_left_t0.size(); i++)
    {
        cv::circle(output_image, cv::Point(points_left_t0[i].x, points_left_t0[i].y), radius, CV_RGB(0, 255, 0));
    }
    for (int i = 0; i < points_left_t1.size(); i++)
    {
        cv::circle(output_image, cv::Point(points_left_t1[i].x, points_left_t1[i].y), radius, CV_RGB(255, 0, 0));
    }
    for (int i = 0; i < points_left_t1.size(); i++)
    {
        cv::line(output_image, points_left_t0[i], points_left_t1[i], CV_RGB(0, 255, 0));
    }

    return output_image;
}

