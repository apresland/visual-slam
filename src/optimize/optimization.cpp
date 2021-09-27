#include <ceres/ceres.h>
#include "sophus/se3.hpp"
#include <opencv2/core/eigen.hpp>
#include <optimize/optimization.h>

void Optimization::optimize(Map::KeyframesType unordered_keyframes,
                            Map::LandmarksType unordered_landmarks,
                            const cv::Mat &K)
{
    std::cout << "Optimization::optimize - " << unordered_keyframes.size() << " active keyframes" << std::endl;
    std::map<unsigned long, std::shared_ptr<Frame>> keyframes(unordered_keyframes.begin(), unordered_keyframes.end());

    Eigen::Matrix3d intrinsics;
    cv::cv2eigen(K, intrinsics);

    ceres::Problem problem;
    ceres::LocalParameterization* se3_parameterization = new SE3Parameterization;

    std::map<size_t, Sophus::SE3d> ceres_poses;
    for(const auto& keyframe : keyframes) {
        ceres_poses[keyframe.second->getID()] = keyframe.second->getPose().inverse().inverse();
        problem.AddParameterBlock(ceres_poses[keyframe.second->getID()].data(), Sophus::SE3d::num_parameters, se3_parameterization);
    }

    std::map<unsigned long, std::shared_ptr<MapPoint>> landmarks;
    for(const auto& keyframe : keyframes)
    {
        for (auto &landmark : keyframe.second->getLandmarks())
        {
            landmarks[landmark->getID()] = landmark;
        }
    }

    Eigen::Vector3d coordinates[landmarks.size()];
    int i = 0;
    for (auto& landmark : landmarks) {
        const cv::Point3f& p3d = landmark.second->getPoint3D();
        Eigen::Vector3d point(p3d.x, p3d.y, p3d.z);
        coordinates[i] = point;
        problem.AddParameterBlock(coordinates[i].data(), 3);
        for (const auto& observation : landmark.second->getObservations()) {
            ReprojectionError* constraint = new ReprojectionError(observation->getPoint2D().x, observation->getPoint2D().y, intrinsics);
            auto cost_func_numeric = new ceres::NumericDiffCostFunction<ReprojectionError, ceres::CENTRAL, 2, 7, 3>(constraint);
            problem.AddResidualBlock(cost_func_numeric,
                                     nullptr /* squared loss */,
                                     ceres_poses[observation->getFrameID()].data(),
                                     coordinates[i].data());
            if(observation->getFrameID() < 2) {
                problem.SetParameterBlockConstant(ceres_poses[observation->getFrameID()].data());
            }
        }
        i += 1;
    }

    std::cout << "Optimization::optimize - solving" << std::endl;

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.max_num_iterations = 10;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";

    i = 0;
    for (auto& landmark :  landmarks) {
        cv::Point3f& p3d = landmark.second->getPoint3D();
        p3d.x = coordinates[i].x();
        p3d.y = coordinates[i].y();
        p3d.z = coordinates[i].z();
        i += 1;
    }

    for(auto& keyframe : keyframes) {
        const auto ceres_pose = ceres_poses[keyframe.second->getID()];
        keyframe.second->setPose(ceres_pose.inverse());
    }

    std::cout << "Optimization::optimize - completed" << std::endl;
}