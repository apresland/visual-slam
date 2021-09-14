#include <vector>
#include <memory>
#include "tracker.h"

void Tracker::track(std::shared_ptr<Frame> frame) {
    std::cout << "[INFO] Tracker::match_right - " << frame->features_left_.size() << " 2D points" << std::endl;

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

            deletion_correction++;
        }
    }

    frame->features_left_.clear();
    for ( auto & pt : frame_t0_points_left ) {
        frame->features_left_.push_back(std::make_shared<Feature>(frame, pt));
    }

    frame->features_right_.clear();
    for ( auto &pt : frame_t0_points_right ) {
        frame->features_right_.push_back(std::make_shared<Feature>(frame, pt));
    }
}

void Tracker::track(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {

    std::cout << "[INFO] Tracker::track - tracking " << frame_t0->features_left_.size() << " 2D points" << std::endl;

    std::vector<cv::Point2f> frame_t0_points_left = frame_t0->get_points_left();
    std::vector<cv::Point2f> frame_t0_points_right = frame_t0->get_points_right();
    std::vector<cv::Point3f> frame_t0_points_3d = frame_t0->get_points_3d();

    std::vector<cv::Point2f> frame_t1_points_left = frame_t1->get_points_left();
    std::vector<cv::Point2f> frame_t1_points_right = frame_t1->get_points_right();
    std::vector<cv::Point3f> frame_t1_points_3d = frame_t1->get_points_3d();


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
            frame_t0_points_3d.erase (frame_t0_points_3d.begin() + (i - deletion_correction));
            frame_t1_points_left.erase (frame_t1_points_left.begin() + (i - deletion_correction));
            frame_t1_points_right.erase (frame_t1_points_right.begin() + (i - deletion_correction));
            frame_t1_points_3d.erase (frame_t1_points_3d.begin() + (i - deletion_correction));

            deletion_correction++;
        }
    }

    frame_t0->features_left_.clear();
    for ( int i =0; i < frame_t0_points_left.size(); i++ ) {
        cv::Point2f pt2d = frame_t0_points_left[i];
        cv::Point3f pt3d = frame_t0_points_3d[i];
        frame_t0->features_left_.push_back(std::make_shared<Feature>(frame_t0, pt2d, pt3d));
    }

    frame_t0->features_right_.clear();
    for ( int i =0; i < frame_t0_points_right.size(); i++ ) {
        cv::Point2f pt2d = frame_t0_points_right[i];
        cv::Point3f pt3d = frame_t0_points_3d[i];
        frame_t0->features_right_.push_back(std::make_shared<Feature>(frame_t0, pt2d, pt3d));
    }

    frame_t1->features_left_.clear();
    for ( int i =0; i < frame_t1_points_left.size(); i++ ) {
        cv::Point2f pt2d = frame_t1_points_left[i];
        cv::Point3f pt3d = frame_t0_points_3d[i];
        frame_t1->features_left_.push_back(std::make_shared<Feature>(frame_t1, pt2d, pt3d));
    }

    frame_t1->features_right_.clear();
    for ( int i =0; i < frame_t1_points_right.size(); i++ ) {
        cv::Point2f pt2d = frame_t1_points_right[i];
        cv::Point3f pt3d = frame_t0_points_3d[i];
        frame_t1->features_right_.push_back(std::make_shared<Feature>(frame_t1, pt2d, pt3d));
    }

    std::cout << "[INFO] Tracker::track - tracked " << frame_t0->features_left_.size() << " 2D points" << std::endl;
}

