#include <string>
#include <thread>
#include "odometry.h"
#include "frame.h"

const std::string KEYPOINT_WINDOW_NAME = "Keypoints";

Odometry::Odometry(std::string path_to_sequence)
    : path_to_sequence_(path_to_sequence) {}

bool Odometry::Init() {
    cv::namedWindow(KEYPOINT_WINDOW_NAME);
    sequence_ = std::make_shared<Sequence>(path_to_sequence_);
    sequence_->Init();
    initialized_ = true;
}

void Odometry::Run() {
    std::shared_ptr<Frame> prev_frame = nullptr;
    for (std::shared_ptr<Frame> frame : sequence_->frames_)
    {
        if(prev_frame) {
            const auto delay_seconds = frame->timestamp_ - prev_frame->timestamp_;
            if (0.0 < delay_seconds) {
                const auto  microseconds = static_cast<unsigned int>(delay_seconds * 1e6);
                std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
            }
        }
        cv::Mat image = cv::imread(frame->image_left_,cv::IMREAD_GRAYSCALE);
        cv::imshow(KEYPOINT_WINDOW_NAME, image);
        cv::waitKey(1);
        prev_frame = frame;
    }
}