#include <memory>
#include <thread>
#include <fstream>
#include <cv_bridge/cv_bridge.h>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>
#include <geometry_msgs/PoseStamped.h>
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
    pose_pub_ = node_handle_.advertise<geometry_msgs::PoseStamped>("pose", 1);
    path_publisher_ = node_handle_.advertise<nav_msgs::Path>("path", 1);
    path_publisher_truth_ = node_handle_.advertise<nav_msgs::Path>("path_truth", 1);
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

    loadGroundTruthPoses();

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
        Sophus::SE3d pose = frontend->getPose();
        publishPose(pose);

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

void System::publishPose(Sophus::SE3d pose) {

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.frame_id
        = "base_link";

    pose_msg.pose.position.x = pose.translation().x();
    pose_msg.pose.position.y = pose.translation().z();
    pose_msg.pose.position.z = pose.translation().y();

    pose_msg.pose.orientation.x = 0.0;
    pose_msg.pose.orientation.y = 0.0;
    pose_msg.pose.orientation.z = 0.0;
    pose_msg.pose.orientation.w = 0.0;

    pose_pub_.publish(pose_msg);

    path_msg_.header.frame_id = pose_msg.header.frame_id;
    path_msg_.poses.emplace_back(pose_msg);
    path_publisher_.publish(path_msg_);
    path_publisher_truth_.publish(path_msg_truth_);
}

void System::loadGroundTruthPoses() {

    std::string filepath = "/data/kitti/odometry/dataset/poses/";
    const std::string filename = filepath + "/00.txt";
    std::vector<cv::Mat> poses;

    std::ifstream file;
    file.open(filename);
    if (!file) {
        throw std::runtime_error("Could not load timestamps file: " + filepath);
    }
    poses.clear();
    std::string  line;
    while (std::getline(file, line)) {
        std::istringstream input(line);
        cv::Mat pose = cv::Mat_<double>(3, 4);
        input >> pose.at<double>(0);
        input >> pose.at<double>(1);
        input >> pose.at<double>(2);
        input >> pose.at<double>(3);
        input >> pose.at<double>(4);
        input >> pose.at<double>(5);
        input >> pose.at<double>(6);
        input >> pose.at<double>(7);
        input >> pose.at<double>(8);
        input >> pose.at<double>(9);
        input >> pose.at<double>(10);
        input >> pose.at<double>(11);

        geometry_msgs::PoseStamped pose_msg;
        pose_msg.header.frame_id
                = "base_link";

        pose_msg.pose.position.x = pose.at<double>(3);
        pose_msg.pose.position.y = pose.at<double>(11);
        pose_msg.pose.position.z = pose.at<double>(7);

        pose_msg.pose.orientation.x = 0.0;
        pose_msg.pose.orientation.y = 0.0;
        pose_msg.pose.orientation.z = 0.0;
        pose_msg.pose.orientation.w = 0.0;

        path_msg_truth_.header.frame_id = pose_msg.header.frame_id;
        path_msg_truth_.poses.emplace_back(pose_msg);
    }
    file.close();
}