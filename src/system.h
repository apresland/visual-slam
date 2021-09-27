//
// Created by andy on 8/9/21.
//

#ifndef VISUAL_SLAM_SYSTEM_H
#define VISUAL_SLAM_SYSTEM_H

#include <memory>
#include <Eigen/Core>
#include "ros/ros.h"
#include "sequence.h"
#include "sensor/camera.h"

class System {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    System(std::string path_to_sequence);
    bool Init();
    void Run();
private:
    std::string path_root_;
    std::string calibration_file_;
    std::shared_ptr<Sequence> sequence_ = nullptr;
    std::vector<std::shared_ptr<Camera>> cameras_;
    bool initialized_ = false;
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "slam_node");
    std::string path_to_sequence = "/data/kitti/odometry/dataset/sequences/00/";
    std::shared_ptr<System> odometry = std::make_shared<System>(path_to_sequence);
    odometry->Init();
    odometry->Run();
    return 0;
}

#endif //VISUAL_SLAM_SYSTEM_H
