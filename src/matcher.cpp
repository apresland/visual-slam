#include <vector>
#include <memory>
#include "frame.h"
#include "feature.h"
#include "matcher.h"

void Matcher::match(std::shared_ptr<Frame> frame) {
    std::cout << "[INFO] Matcher::match_right - " << frame->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_left = frame->get_points_left();
    std::vector<cv::Point2f> frame_t0_points_right = frame->get_points_right();

    std::vector<float> err;
    std::vector<uchar> status;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    // matching with LK optical flow
    calcOpticalFlowPyrLK(frame->image_left_, frame->image_right_, frame_t0_points_left, frame_t0_points_right, status, err, window_size, 3, term_criteria, 0, 0.001);

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
            frame->features_left_.erase (frame->features_left_.begin() + (i - deletion_correction));

            deletion_correction++;
        }
    }

    frame->features_right_.clear();
    for ( auto &pt : frame_t0_points_right ) {
        frame->features_right_.push_back(std::make_shared<Feature>(frame, pt));
    }
}

void Matcher::match(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {

    std::cout << "[INFO] Matcher::match - matching " << frame_t0->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_left = frame_t0->get_points_left();
    std::vector<cv::Point2f> frame_t0_points_right = frame_t0->get_points_right();

    std::vector<cv::Point2f> frame_t1_points_left = frame_t1->get_points_left();
    std::vector<cv::Point2f> frame_t1_points_right = frame_t1->get_points_right();

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    std::vector<uchar> status_0;
    std::vector<uchar> status_1;
    std::vector<uchar> status_2;
    std::vector<uchar> status_3;

    // circular matching with LK optical flow
    calcOpticalFlowPyrLK(frame_t0->image_left_, frame_t1->image_left_, frame_t0_points_left, frame_t1_points_left, status_0, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(frame_t1->image_left_, frame_t1->image_right_, frame_t1_points_left, frame_t1_points_right, status_1, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(frame_t1->image_right_, frame_t0->image_right_, frame_t1_points_right, frame_t0_points_right, status_2, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(frame_t0->image_right_, frame_t0->image_left_, frame_t0_points_right, frame_t0_points_left, status_3, err, window_size, 3, term_criteria, 0, 0.001);

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
            frame_t0->features_left_.erase(frame_t0->features_left_.begin() + (i - deletion_correction));
            deletion_correction++;
        }
    }

    frame_t0->features_right_.clear();
    for ( int i =0; i < frame_t0_points_right.size(); i++ ) {
        cv::Point2f pt2d = frame_t0_points_right[i];
        std::shared_ptr<Feature> feature = std::make_shared<Feature>(frame_t0, pt2d);
        frame_t0->features_right_.push_back(feature);
    }

    std::cout << "[INFO] Matcher::match - matched " << frame_t0->features_right_.size() << " 2D points" << std::endl;
}

void Matcher::track(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {
    std::cout << "[INFO] Matcher::track - tracking " << frame_t0->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_left = frame_t0->get_points_left();
    std::vector<cv::Point2f> frame_t1_points_left = frame_t0->get_points_left();

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);
    std::vector<uchar> status;

    calcOpticalFlowPyrLK(frame_t0->image_left_, frame_t1->image_left_, frame_t0_points_left, frame_t1_points_left, status, err, window_size, 3, term_criteria, 0, 0.001);

    std::vector<cv::Point2f> points_left_t0_visual;
    std::vector<cv::Point2f> points_left_t1_visual;

    frame_t1->features_left_.clear();
    for(int i=0; i < status.size(); i++) {
        if(status[i]) {
            cv::Point2f p2d_t1 = frame_t1_points_left[i];
            std::shared_ptr<Feature> feature_t1 = std::make_shared<Feature>(frame_t1, p2d_t1);
            feature_t1->landmark_ = frame_t0->features_left_[i]->landmark_;
            frame_t1->features_left_.push_back(feature_t1);
            points_left_t0_visual.push_back(frame_t0_points_left[i]);
            points_left_t1_visual.push_back(frame_t1_points_left[i]);
        }
    }

    if (viewer_) {
        viewer_->display_tracking(points_left_t0_visual, points_left_t1_visual, frame_t1->image_left_);
    }
    std::cout << "[INFO] Matcher::track - tracked " << frame_t1->features_left_.size() << " 2D points" << std::endl;
}
