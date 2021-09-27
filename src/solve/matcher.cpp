#include <vector>
#include <memory>
#include <sensor/feature.h>
#include <sensor/frame.h>
#include "matcher.h"

void Matcher::match_stereo(Context &context) {
    std::cout << "[INFO] Matcher::match_right - " << context.frame_current_->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_left = context.frame_current_->getPointsLeft();
    std::vector<cv::Point2f> frame_t0_points_right = context.frame_current_->getPointsRight();

    std::vector<float> err;
    std::vector<uchar> status;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    // matching with LK optical flow
    calcOpticalFlowPyrLK(context.frame_current_->image_left_, context.frame_current_->image_right_, frame_t0_points_left, frame_t0_points_right, status, err, window_size, 3, term_criteria, 0, 0.001);

    // filter out bad matches
    int deletion_correction = 0;
    for(int i=0; i < status.size(); i++)
    {
        // get next points respecting prior deletions
        cv::Point2f point = frame_t0_points_right.at(i - deletion_correction);

        // check for failed KLT or out-of-frame points
        if ((status.at(i) == 0) || (point.x < 0) || (point.y < 0) )
        {
            // erase bad matches
            frame_t0_points_left.erase (frame_t0_points_left.begin() + (i - deletion_correction));
            frame_t0_points_right.erase (frame_t0_points_right.begin() + (i - deletion_correction));
            context.frame_current_->features_left_.erase (context.frame_current_->features_left_.begin() + (i - deletion_correction));

            deletion_correction++;
        }
    }

    context.frame_current_->features_right_.clear();
    for ( auto &pt : frame_t0_points_right ) {
        std::shared_ptr<Feature> feature = std::make_shared<Feature>(context.frame_current_->getID(), pt);
        context.frame_current_->features_right_.push_back(feature);
    }
}

void Matcher::match_quadro(Context &context) {

    std::cout << "[INFO] Matcher::match_stereo - matching " << context.frame_previous_->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_left = context.frame_previous_->getPointsLeft();
    std::vector<cv::Point2f> frame_t0_points_right = context.frame_previous_->getPointsRight();

    std::vector<cv::Point2f> frame_t1_points_left = context.frame_current_->getPointsLeft();
    std::vector<cv::Point2f> frame_t1_points_right = context.frame_current_->getPointsRight();

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    std::vector<uchar> status_0;
    std::vector<uchar> status_1;
    std::vector<uchar> status_2;
    std::vector<uchar> status_3;

    // circular matching with LK optical flow
    calcOpticalFlowPyrLK(context.frame_previous_->image_left_, context.frame_current_->image_left_, frame_t0_points_left, frame_t1_points_left, status_0, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(context.frame_current_->image_left_, context.frame_current_->image_right_, frame_t1_points_left, frame_t1_points_right, status_1, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(context.frame_current_->image_right_, context.frame_previous_->image_right_, frame_t1_points_right, frame_t0_points_right, status_2, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(context.frame_previous_->image_right_, context.frame_previous_->image_left_, frame_t0_points_right, frame_t0_points_left, status_3, err, window_size, 3, term_criteria, 0, 0.001);

    // filter out bad matches
    int deletion_correction = 0;
    for(int i=0; i < status_3.size(); i++)
    {
        // get next points respecting prior deletions
        cv::Point2f point_0 = frame_t0_points_left.at(i - deletion_correction);
        cv::Point2f point_1 = frame_t0_points_right.at(i - deletion_correction);
        cv::Point2f point_2 = frame_t1_points_left.at(i - deletion_correction);
        cv::Point2f point_3 = frame_t1_points_right.at(i - deletion_correction);

        // check for failed KLT or out-of-frame points
        if ((status_3.at(i) == 0) || (point_3.x < 0) || (point_3.y < 0) ||
            (status_2.at(i) == 0) || (point_2.x < 0) || (point_2.y < 0) ||
            (status_1.at(i) == 0) || (point_1.x < 0) || (point_1.y < 0) ||
            (status_0.at(i) == 0) || (point_0.x < 0) || (point_0.y < 0))
        {
            // erase bad matches
            frame_t0_points_left.erase (frame_t0_points_left.begin() + (i - deletion_correction));
            frame_t0_points_right.erase (frame_t0_points_right.begin() + (i - deletion_correction));
            frame_t1_points_left.erase (frame_t1_points_left.begin() + (i - deletion_correction));
            frame_t1_points_right.erase (frame_t1_points_right.begin() + (i - deletion_correction));
            context.frame_previous_->features_left_.erase(context.frame_previous_->features_left_.begin() + (i - deletion_correction));
            deletion_correction++;
        }
    }

    context.frame_previous_->features_right_.clear();
    for ( int i =0; i < frame_t0_points_right.size(); i++ ) {
        cv::Point2f pt2d = frame_t0_points_right[i];
        std::shared_ptr<Feature> feature = std::make_shared<Feature>(context.frame_previous_->getID(), pt2d);
        context.frame_previous_->features_right_.push_back(feature);
    }

    std::cout << "[INFO] Matcher::match_stereo - matched " << context.frame_previous_->features_right_.size() << " 2D points" << std::endl;
}
