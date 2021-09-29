#include <memory>
#include <thread>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include "system.h"
#include "viewer.h"
#include "frontend.h"
#include "backend.h"
#include "solve/matcher.h"
#include "solve/triangulator.h"
#include "solve/estimation.h"

System::System(ros::NodeHandle& node_handle)
    : node_handle_(node_handle)
    , image_trans_(node_handle_)
    , path_root_("/data/kitti/odometry/dataset/sequences/00/")
    , calibration_file_(path_root_ + "/calib.txt") {
    image_pub_00_ = image_trans_.advertiseCamera("grayscale/left/image", 1);
    image_pub_01_ = image_trans_.advertiseCamera("grayscale/right/image", 1);
}

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

    std::shared_ptr<Viewer> viewer;
    //viewer = std::make_shared<Viewer>();
    if (viewer) {
        viewer->init();
        viewer->loadGroundTruthPoses();
    }

    std::shared_ptr<Matcher> matcher = std::make_shared<Matcher>();
    std::shared_ptr<Tracker> tracker = std::make_shared<Tracker>();
    std::shared_ptr<Estimation> estimation = std::shared_ptr<Estimation>();
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

    cv_bridge::CvImage cv_bridge;;
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

        cv::Mat image_00
            = cv::imread(element->image_00_, cv::IMREAD_GRAYSCALE);
        sensor_msgs::ImagePtr image_msg_00
            = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_00).toImageMsg();
        image_msg_00->header.frame_id
            = "base_link";

        cv::Mat image_01
                = cv::imread(element->image_01_, cv::IMREAD_GRAYSCALE);
        sensor_msgs::ImagePtr image_msg_01
                = cv_bridge::CvImage(std_msgs::Header(), "mono8", image_01).toImageMsg();
        image_msg_01->header.frame_id
                = "base_link";

        publishLeftImage(image_msg_00);
        publishRightImage(image_msg_01);

        frontend->pushback(image_00, image_01);
        ros::spinOnce();
    }

    backend->terminate();
}

void System::publishLeftImage(const sensor_msgs::ImagePtr &image) {

    sensor_msgs::CameraInfoPtr
        camera_info_msg(new sensor_msgs::CameraInfo());
    camera_info_msg->header.frame_id
        = image->header.frame_id;

    image_pub_00_.publish(image, camera_info_msg);
}

void System::publishRightImage(const sensor_msgs::ImagePtr &image) {

    sensor_msgs::CameraInfoPtr
            camera_info_msg(new sensor_msgs::CameraInfo());
    camera_info_msg->header.frame_id
            = image->header.frame_id;

    image_pub_01_.publish(image, camera_info_msg);
}