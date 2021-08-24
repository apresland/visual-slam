#ifndef VISUAL_SLAM_FRAME_H
#define VISUAL_SLAM_FRAME_H

#include <vector>
#include <memory>
#include <Eigen/Core>
#include <sophus/se3.hpp>
#include <opencv2/core/core.hpp>

struct Feature;
struct Frame {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    Frame(){};
    cv::Mat image_left_;
    cv::Mat image_right_;
    cv::Mat disparity_;
    std::vector<cv::KeyPoint> keypoints_left_;
    std::vector<cv::KeyPoint> keypoints_right_;
    std::vector<std::shared_ptr<Feature>> features_left_;
    std::vector<std::shared_ptr<Feature>> features_right_;
    cv::Mat descriptors_left_;
    cv::Mat descriptors_right_;
    std::vector<cv::DMatch> matches_;
    std::vector<cv::DMatch> matches_left_;
    std::vector<cv::DMatch> matches_right_;
    double timestamp_;
    Sophus::SE3d pose_;
};


#endif //VISUAL_SLAM_FRAME_H
