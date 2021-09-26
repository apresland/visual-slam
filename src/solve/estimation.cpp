#include <sensor/frame.h>
#include "estimation.h"

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera getPose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
void Estimation::estimate(std::shared_ptr<Frame> frame_previous,
                          std::shared_ptr<Frame> frame_current,
                          const cv::Mat K)
{
    std::vector<cv::Point2f> points_2d = frame_current->getPointsLeft();
    std::cout << "[INFO] Frontend::estimate_pose - points { 2D " << points_2d.size() << " }" << std::endl;
    std::vector<cv::Point3f> points_3d = frame_current->getPoints3D();
    std::cout << "[INFO] Frontend::estimate_pose - points { 3D " << points_3d.size() << " }" << std::endl;

    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac(points_3d, points_2d, K, coeffs, rvec, tvec,
                       false, 100, 1.0, 0.999, inliers );
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Rodrigues(rvec, R);

    // Relative motion (T)
    Eigen::Matrix3d SO3_R;
    Eigen::Vector3d SO3_t;
    SO3_R << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    SO3_t << tvec.at<double>(0, 0), tvec.at<double>(1, 0), tvec.at<double>(2, 0);

    Sophus::SE3d T = Sophus::SE3d(SO3_R, SO3_t);

    removeOutliers(frame_current, inliers);

    double angle = T.rotationMatrix().norm();
    double distance = T.translation().norm();
    std::cout << "[INFO] Frontend::esimate_pose - relative motion = " << distance << " angle = " << angle << std::endl;

    // Transform World-to-Camera
    Sophus::SE3d T_c_w = frame_previous->getPose();
    if (distance > 0.05 && distance < 5) {
        frame_current->setIsKeyframe(true);
        T_c_w = T_c_w * T.inverse();
    } else {
        frame_current->setIsKeyframe(true);
        std::cout << "[WARNING] getPose not updated due to out-of-bounds scale value" << distance << std::endl;
    }

    frame_current->setPose(T_c_w);
}

void Estimation::removeOutliers(std::shared_ptr<Frame> frame, cv::Mat inliers) {

    for (int idx = 0; idx < inliers.rows; idx++)
    {
        int index = inliers.at<int>(idx, 0);
        frame->features_left_[index]->isInlier(true);
    }

    std::vector<std::shared_ptr<Feature>> features_left(frame->features_left_);
    frame->features_left_.clear();
    for ( int i =0; i < features_left.size(); i++)
    {
        if ( features_left[i]->getIsInlier()) frame->features_left_.push_back(features_left[i]);
    }
}