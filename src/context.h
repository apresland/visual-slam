#ifndef STEREO_CONTEXT_H
#define STEREO_CONTEXT_H

#include <sensor/frame.h>

struct Context {

    std::shared_ptr<Frame> frame_previous_{nullptr};
    std::shared_ptr<Frame> frame_current_{nullptr};

    std::vector<cv::KeyPoint> keypoints_;
    std::vector<cv::Point2f> flow_points_2d_t0;
    std::vector<cv::Point2f> flow_points_2d_t1;

    size_t frame_id_{0};
    void pushback(cv::Mat &image_left, cv::Mat &image_right) {
        std::unique_ptr<Frame> frame = std::make_unique<Frame>();
        frame->image_left_ = image_left;
        frame->image_right_ = image_right;
        frame->setID(frame_id_);
        frame_previous_ = std::move(frame_current_);
        frame_current_ = std::move(frame);
        frame_id_++;
    }
};

#endif //STEREO_CONTEXT_H
