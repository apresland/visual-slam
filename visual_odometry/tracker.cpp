#include "tracker.h"

void Tracker::track(cv::Mat imgL_t0, cv::Mat imgR_t0, cv::Mat imgL_t1, cv::Mat imgR_t1,
                    std::vector<cv::Point2f> &ptsL_t0, std::vector<cv::Point2f> &ptsR_t0,
                    std::vector<cv::Point2f> &ptsL_t1, std::vector<cv::Point2f> &ptsR_t1,
                    std::vector<cv::Point2f> &ptsL_t0_return) {

    std::vector<float> err;
    cv::Size window_size=cv::Size(21,21);
    cv::TermCriteria term_criteria = cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 30, 0.01);

    std::vector<uchar> status_0;
    std::vector<uchar> status_1;
    std::vector<uchar> status_2;
    std::vector<uchar> status_3;

    // circular matching with LK optical flow
    calcOpticalFlowPyrLK(imgL_t0, imgL_t1, ptsL_t0, ptsL_t1, status_0, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(imgL_t1, imgR_t1, ptsL_t1, ptsR_t1, status_1, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(imgR_t1, imgR_t0, ptsR_t1, ptsR_t0, status_2, err, window_size, 3, term_criteria, 0, 0.001);
    calcOpticalFlowPyrLK(imgR_t0, imgL_t0, ptsR_t0, ptsL_t0_return, status_3, err, window_size, 3, term_criteria, 0, 0.001);

    // filter out bad matches
    int deletion_correction = 0;
    for(int i=0; i < status_3.size(); i++)
    {
        // get next points respecting prior deletions
        cv::Point2f point_0 = ptsL_t0.at(i - deletion_correction);
        cv::Point2f point_1 = ptsR_t0.at(i - deletion_correction);
        cv::Point2f point_2 = ptsL_t1.at(i - deletion_correction);
        cv::Point2f point_3 = ptsR_t1.at(i - deletion_correction);

        // check for failed KLT or out-of-frame points
        if ((status_3.at(i) == 0) || (point_3.x < 0) || (point_3.y < 0) ||
            (status_2.at(i) == 0) || (point_2.x < 0) || (point_2.y < 0) ||
            (status_1.at(i) == 0) || (point_1.x < 0) || (point_1.y < 0) ||
            (status_0.at(i) == 0) || (point_0.x < 0) || (point_0.y < 0))
        {
            // erase bad matches
            ptsL_t0.erase (ptsL_t0.begin() + (i - deletion_correction));
            ptsR_t0.erase (ptsR_t0.begin() + (i - deletion_correction));
            ptsL_t1.erase (ptsL_t1.begin() + (i - deletion_correction));
            ptsR_t1.erase (ptsR_t1.begin() + (i - deletion_correction));
            ptsL_t0_return.erase (ptsL_t0_return.begin() + (i - deletion_correction));

            deletion_correction++;
        }
    }
}
