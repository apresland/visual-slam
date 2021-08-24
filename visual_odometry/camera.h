#ifndef VISUAL_SLAM_CAMERA_H
#define VISUAL_SLAM_CAMERA_H

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

class Camera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Camera();
    Camera(double fx, double fy, double cx, double cy, double bf, Sophus::SE3d pose)
        : fx_(fx), fy_(fy), cx_(cx), cy_(cy), bf_(bf), pose_(pose) {
        pose_inverse_ = pose_.inverse();
    }

    double fx_{0.0};
    double fy_{0.0};
    double cx_{0.0};
    double cy_{0.0};
    double bf_{0.0};

    Sophus::SE3d pose_;
    Sophus::SE3d pose_inverse_;

    cv::Mat  K() const {
        cv::Mat k = (cv::Mat_<double>(3, 3) << fx_, 0, cx_, 0, fy_, cy_, 0, 0, 1);
        return k;
    }

    cv::Mat P() const {
        cv::Mat p = (cv::Mat_<double>(3, 4) << fx_, 0, cx_, bf_, 0, fy_, cy_, 0, 0,  0, 1, 0);
        return p;
    }

    cv::Point3d pixel2camera(const cv::Point2d &p, double depth = 1);
};

#endif //VISUAL_SLAM_CAMERA_H
