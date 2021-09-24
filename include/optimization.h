#ifndef VISUAL_SLAM_BA_CERES_H
#define VISUAL_SLAM_BA_CERES_H

#include <ceres/local_parameterization.h>
#include <sophus/se3.hpp>
#include "map.h"

class Optimization {
public:
    void optimize(Map::KeyframesType keyframes, Map::LandmarksType landmarks, const cv::Mat &K);
};

class SE3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~SE3Parameterization() {}

    virtual bool Plus(double const* T_raw, double const* delta_raw,
                      double* T_plus_delta_raw) const {
        Eigen::Map<Sophus::SE3d const> const T(T_raw);
        Eigen::Map<Sophus::Vector6d const> const delta(delta_raw);
        Eigen::Map<Sophus::SE3d> T_plus_delta(T_plus_delta_raw);
        T_plus_delta = T * Sophus::SE3d::exp(delta);
        return true;
    }

    virtual bool ComputeJacobian(double const* T_raw,
                                 double* jacobian_raw) const {
        Eigen::Map<Sophus::SE3d const> T(T_raw);
        Eigen::Map<Eigen::Matrix<double, 7, 6, Eigen::RowMajor>> jacobian(jacobian_raw);
        jacobian = T.Dx_this_mul_exp_x_at_0();
        return true;
    }

    virtual int GlobalSize() const { return Sophus::SE3d::num_parameters; }

    virtual int LocalSize() const { return Sophus::SE3d::DoF; }
};

class ReprojectionError {
public:
    ReprojectionError(double observed_x, double observed_y, const Eigen::Matrix3d& intrinsics) :
            observed_x_{observed_x}, observed_y_{observed_y}, intrinsics_{intrinsics} {}

    template <typename T>
    bool operator()(const T* const extrinsics, const T* const point, T* residuals) const
    {
        Sophus::SE3d se3;
        for(int i = 0; i < 7; ++i)
            se3.data()[i] = extrinsics[i];

        Eigen::Vector4d vec {point[0], point[1], point[2], 1.0};
        Eigen::Vector4d p = se3.matrix() * vec;

        T predicted_x = p[0] / p[2] * intrinsics_(0, 0) + intrinsics_(0, 2);
        T predicted_y = p[1] / p[2] * intrinsics_(1, 1) + intrinsics_(1, 2);

        residuals[0] = predicted_x - T(observed_x_);
        residuals[1] = predicted_y - T(observed_y_);

        return true;
    }

private:
    double observed_x_, observed_y_;
    const Eigen::Matrix3d intrinsics_;
};

#endif //VISUAL_SLAM_BA_CERES_H
