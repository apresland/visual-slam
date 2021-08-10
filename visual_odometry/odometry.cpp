#include <string>
#include <thread>
#include "odometry.h"
#include "frame.h"
#include "viewer.h"
#include "frontend.h"

Odometry::Odometry(std::string path_to_sequence)
    : path_to_sequence_(path_to_sequence) {}

bool Odometry::Init() {
    sequence_ = std::make_shared<Sequence>(path_to_sequence_);
    sequence_->Init();
    initialized_ = true;
}

void Odometry::Run() {

    std::shared_ptr<Frontend> frontend = std::make_shared<Frontend>();
    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>();
    frontend->SetViewer(viewer);
    viewer->init();

    std::shared_ptr<Sequence::StereoPair> prev_element = nullptr;
    for (std::shared_ptr<Sequence::StereoPair> element : sequence_->elements_)
    {
        if(prev_element) {
            const auto delay_seconds = element->timestamp_ - prev_element->timestamp_;
            if (0.0 < delay_seconds) {
                const auto  microseconds = static_cast<unsigned int>(delay_seconds * 1e6);
                std::this_thread::sleep_for(std::chrono::microseconds(microseconds));
            }
        }

        cv::Mat image_left = cv::imread(element->image_left_,cv::IMREAD_GRAYSCALE);
        cv::Mat image_right = cv::imread(element->image_right_,cv::IMREAD_GRAYSCALE);

        cv::Mat image_left_resized, image_right_resized;
        cv::resize(image_left, image_left_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);
        cv::resize(image_right, image_right_resized, cv::Size(), 0.5, 0.5,
                   cv::INTER_NEAREST);

        std::shared_ptr<Frame> frame = std::make_shared<Frame>();
        frame->image_left_ = image_left_resized;
        frame->image_right_ = image_right_resized;
        frame->timestamp_ = element->timestamp_;

        frontend->process(frame);
    }
}