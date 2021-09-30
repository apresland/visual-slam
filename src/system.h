//
// Created by andy on 8/9/21.
//

#ifndef VISUAL_SLAM_SYSTEM_H
#define VISUAL_SLAM_SYSTEM_H

#include <memory>
#include <Eigen/Core>
#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include "nav_msgs/Path.h"
#include "sequence.h"
#include "sensor/camera.h"

class System {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    System(ros::NodeHandle& nodeHandle);
    bool Init();
    void Run();
private:

    void loadGroundTruthPoses();

    void publishLeftImage(const sensor_msgs::ImagePtr &image);
    void publishRightImage(const sensor_msgs::ImagePtr &image);
    void publishPose(Sophus::SE3d pose);

    std::string path_root_;
    std::string calibration_file_;
    std::shared_ptr<Sequence> sequence_ = nullptr;
    std::vector<std::shared_ptr<Camera>> cameras_;
    ros::NodeHandle& node_handle_;
    image_transport::ImageTransport image_trans_;
    image_transport::CameraPublisher image_pub_00_;
    image_transport::CameraPublisher image_pub_01_;
    nav_msgs::Path path_msg_;
    nav_msgs::Path path_msg_truth_;
    ros::Publisher pose_pub_;
    ros::Publisher path_publisher_;
    ros::Publisher path_publisher_truth_;
    bool initialized_ = false;

    std::vector<cv::Mat> ground_truth_poses_;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "stereo_odometer_node");
    ros::NodeHandle node_handle("stereo_odometer");
    std::shared_ptr<System> odometry = std::make_shared<System>(node_handle);
    odometry->Init();
    odometry->Run();
    return 0;
}

#endif //VISUAL_SLAM_SYSTEM_H
