#include "frame.h"
#include "estimation.h"

// 3D-to-2D: Motion from 3D structure and 2D image feature correspondence.
// Use the Perspective-n-Point (PnP) algorithm to provide "perspective-from-3-points" (P3P). Estimates
// the camera get_pose (t,r) that minimizes the reprojection error of 3D points onto the 2D image. Convert the
// rotation vector (r) into a rotation matrix (R) with Rodrigues algorithm.
void Estimation::estimate(std::shared_ptr<Frame> frame_previous,
                          std::shared_ptr<Frame> frame_current,
                          const cv::Mat K)
{
    std::vector<cv::Point2f> points_2d = frame_current->get_points_left();
    std::cout << "[INFO] Frontend::estimate_pose - points { 2D " << points_2d.size() << " }" << std::endl;
    std::vector<cv::Point3f> points_3d = frame_current->get_points_3d();
    std::cout << "[INFO] Frontend::estimate_pose - points { 3D " << points_3d.size() << " }" << std::endl;

    cv::Mat inliers;
    cv::Mat coeffs = cv::Mat::zeros(4, 1, CV_64FC1);
    cv::Mat t = cv::Mat::zeros(3, 1, CV_64F);
    cv::Mat r = cv::Mat::zeros(3, 1, CV_64FC1);
    cv::solvePnPRansac(points_3d, points_2d, K, coeffs, r, t,
                       false, 100, 1.0, 0.999, inliers );
    cv::Mat R = cv::Mat::eye(3, 3, CV_64F);
    cv::Rodrigues(r, R);

    // Relative motion (T)
    Eigen::Matrix3d SO3_R;
    Eigen::Vector3d SO3_t;
    SO3_R << R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
            R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
            R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2);
    SO3_t << t.at<double>(0, 0), t.at<double>(1, 0), t.at<double>(2, 0);

    Sophus::SE3d T = Sophus::SE3d(SO3_R, SO3_t);

    remove_outliers(frame_current, inliers);

    double distance = cv::norm(t);
    double angle = cv::norm(R);
    std::cout << "[INFO] Frontend::esimate_pose - relative motion = " << distance << " angle = " << angle << std::endl;

    // Transform World-to-Camera
    Sophus::SE3d T_c_w = frame_previous->get_pose();
    if (distance > 0.05 && distance < 5) {
        frame_current->is_keyframe_ = true;
        T_c_w = T_c_w * T.inverse();
        //T_c_w = T;
    } else {
        frame_current->is_keyframe_ = false;
        std::cout << "[WARNING] get_pose not updated due to out-of-bounds scale value" << distance << std::endl;
    }

    frame_current->set_pose(T_c_w);
}

void Estimation::remove_outliers(std::shared_ptr<Frame> frame, cv::Mat inliers) {

    for (int idx = 0; idx < inliers.rows; idx++)
    {
        int index = inliers.at<int>(idx, 0);
        frame->features_left_[index]->is_inlier_ = true;
    }

    std::vector<std::shared_ptr<Feature>> features_left(frame->features_left_);
    frame->features_left_.clear();
    for ( int i =0; i < features_left.size(); i++)
    {
        if ( features_left[i]->is_inlier_) frame->features_left_.push_back(features_left[i]);
    }
}