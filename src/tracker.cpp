#include <vector>
#include <memory>
#include "tracker.h"
#include "feature.h"

void Tracker::track(std::shared_ptr<Frame> frame_t0, std::shared_ptr<Frame> frame_t1) {

    std::vector<cv::Point2f>  ptsL_t0_return;

    std::vector<cv::Point2f> frame_t0_points_left_ = frame_t0->get_points_left();
    std::vector<cv::Point2f> frame_t0_points_right_ = frame_t0->get_points_right();
    std::vector<cv::Point2f> frame_t1_points_left_ = frame_t1->get_points_left();
    std::vector<cv::Point2f> frame_t1_points_right_ = frame_t1->get_points_right();

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    std::vector<uchar> status_0;
    std::vector<uchar> status_1;
    std::vector<uchar> status_2;
    std::vector<uchar> status_3;

    // circular matching with LK optical flow
    calcOpticalFlowPyrLK(frame_t0->image_left_, frame_t1->image_left_, frame_t0_points_left_, frame_t1_points_left_, status_0, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(frame_t1->image_left_, frame_t1->image_right_, frame_t1_points_left_, frame_t1_points_right_, status_1, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(frame_t1->image_right_, frame_t0->image_right_, frame_t1_points_right_, frame_t0_points_right_, status_2, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(frame_t0->image_right_, frame_t0->image_left_, frame_t0_points_right_, ptsL_t0_return, status_3, err, window_size, 3, term_criteria, 0, 0.001);

    // filter out bad matches
    int deletion_correction = 0;
    for(int i=0; i < status_3.size(); i++)
    {
        // get next points respecting prior deletions
        cv::Point2f point_0 = frame_t0_points_left_.at(i - deletion_correction);
        cv::Point2f point_1 = frame_t0_points_right_.at(i - deletion_correction);
        cv::Point2f point_2 = frame_t1_points_left_.at(i - deletion_correction);
        cv::Point2f point_3 = frame_t1_points_right_.at(i - deletion_correction);

        // check for failed KLT or out-of-frame points
        if ((status_3.at(i) == 0) || (point_3.x < 0) || (point_3.y < 0) ||
            (status_2.at(i) == 0) || (point_2.x < 0) || (point_2.y < 0) ||
            (status_1.at(i) == 0) || (point_1.x < 0) || (point_1.y < 0) ||
            (status_0.at(i) == 0) || (point_0.x < 0) || (point_0.y < 0))
        {
            // erase bad matches
            frame_t0_points_left_.erase (frame_t0_points_left_.begin() + (i - deletion_correction));
            frame_t0_points_right_.erase (frame_t0_points_right_.begin() + (i - deletion_correction));
            frame_t1_points_left_.erase (frame_t1_points_left_.begin() + (i - deletion_correction));
            frame_t1_points_right_.erase (frame_t1_points_right_.begin() + (i - deletion_correction));
            ptsL_t0_return.erase (ptsL_t0_return.begin() + (i - deletion_correction));

            deletion_correction++;
        }
    }

    frame_t0->set_features_left(frame_t0_points_left_);
    frame_t0->set_features_right(frame_t0_points_right_);
    frame_t1->set_features_left(frame_t1_points_left_);
    frame_t1->set_features_right(frame_t1_points_right_);
}

