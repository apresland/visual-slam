#include <memory>
#include <string>
#include <thread>
#include <fstream>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include "system.h"
#include "sensor/frame.h"
#include "viewer.h"
#include "frontend.h"
#include "backend.h"
#include "solve/matcher.h"
#include "solve/triangulator.h"
#include "solve/estimation.h"

System::System(std::string path_to_sequence)
    : path_root_(path_to_sequence)
    , calibration_file_(path_root_ + "/calib.txt"){}

bool System::Init() {
    sequence_ = std::make_shared<Sequence>(path_root_);
    sequence_->Init();
    std::ifstream fin(calibration_file_);
    if (!fin) {return false;}
    for (int i = 0; i < 4; ++i) {
        char camera_name[3];
        for (int k = 0; k < 3; ++k) { fin >> camera_name[k];}
        double P[12];
        for (int k = 0; k < 12; ++k) { fin >> P[k]; }
        std::shared_ptr<Camera> camera = std::make_shared<Camera>(P[0], P[5], P[2], P[6], P[3]);
        cameras_.push_back(camera);
    }
    fin.close();
    initialized_ = true;
}

void System::Run() {

    std::shared_ptr<Matcher> matcher = std::make_shared<Matcher>();
    std::shared_ptr<Tracker> tracker = std::make_shared<Tracker>();
    std::shared_ptr<Estimation> estimation = std::shared_ptr<Estimation>();
    std::shared_ptr<Viewer> viewer = std::make_shared<Viewer>();
    std::shared_ptr<Map> map = std::make_shared<Map>();

    std::shared_ptr<Triangulator> triangulator = std::make_shared<Triangulator>(cameras_[0], cameras_[1]);
    triangulator->setMap(map);

    std::shared_ptr<Backend> backend = std::make_shared<Backend>();
    backend->setCameras(cameras_[0], cameras_[1]);
    backend->setMap(map);

    std::shared_ptr<Frontend> frontend = std::make_shared<Frontend>();
    frontend->setCameras(cameras_[0], cameras_[1]);
    frontend->setMap(map);
    frontend->setMatcher(matcher);
    frontend->setTracker(tracker);
    frontend->setTriangulator(triangulator);
    frontend->setEstimation(estimation);
    frontend->setViewer(viewer);
    frontend->setBackend(backend);
    viewer->init();

    int index = 0;
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

        cv::Mat image_left_resized;
        cv::Mat image_left = cv::imread(element->image_left_,cv::IMREAD_GRAYSCALE);
        cv::resize(image_left, image_left_resized, cv::Size(), IMAGE_SCALE_FACTOR, IMAGE_SCALE_FACTOR,
                   cv::INTER_NEAREST);

        cv::Mat image_right_resized;
        cv::Mat image_right = cv::imread(element->image_right_,cv::IMREAD_GRAYSCALE);
        cv::resize(image_right, image_right_resized, cv::Size(), IMAGE_SCALE_FACTOR, IMAGE_SCALE_FACTOR,
                   cv::INTER_NEAREST);

        frontend->pushback(image_left_resized, image_right_resized);

        ++index;
    }

    backend->terminate();
}