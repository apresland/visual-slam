#include <opencv2/opencv.hpp>
#include "viewer.h"
#include "sensor/feature.h"

const std::string FEATURES_WINDOW_NAME = "Features";
const std::string BUCKETED_FEATURES_WINDOW_NAME = "Bucketed Fetures";
const std::string CIRCULAR_MATCHES_WINDOW_NAME = "Circular Matched Features";
const std::string OPTICAL_FLOW_WINDOW_NAME = "Optical Flow";

Viewer::Viewer() {}

void Viewer::init() {
    cv::namedWindow(FEATURES_WINDOW_NAME);
    cv::namedWindow(BUCKETED_FEATURES_WINDOW_NAME);
    cv::namedWindow(CIRCULAR_MATCHES_WINDOW_NAME);
    cv::namedWindow(OPTICAL_FLOW_WINDOW_NAME);
    //cv::namedWindow(DISPARITY_WINDOW_NAME);
}

void Viewer::update(const std::shared_ptr<Frame> frame_previous,
                    const std::shared_ptr<Frame> frame_current) {

    displayFeatures(frame_current);
    displayTracking(frame_previous, frame_current);
    displayTrajectory(frame_current);
}

void Viewer::displayFeatures(const std::shared_ptr<Frame> frame_current) {

    const cv::Mat &image = frame_current->image_left_;
    std::vector<cv::Point2f> points_left_t1 = frame_current->getPointsLeft();

    cv::Scalar color_g(0, 255, 0), color_b(255, 0, 0), color_r(0, 0, 255);

    std::vector<cv::KeyPoint> keypoints;
    for(auto point : points_left_t1) {
        keypoints.push_back(cv::KeyPoint(point, 1.f));
    }
    cv::Mat img_bucketed;
    cv::drawKeypoints(image, keypoints, img_bucketed, color_g);
    cv::imshow(BUCKETED_FEATURES_WINDOW_NAME, img_bucketed);
    cv::waitKey(1);

    std::vector<cv::KeyPoint> keypoints2;
    for(auto point : points_left_t1) {
        keypoints2.push_back(cv::KeyPoint(point, 1.f));
    }
    cv::Mat img_matches;
    cv::drawKeypoints(image, keypoints2, img_matches, color_g);
    cv::imshow(CIRCULAR_MATCHES_WINDOW_NAME, img_matches);
    cv::waitKey(1);
}

void Viewer::displayTracking(const std::shared_ptr<Frame> frame_previous,
                             const std::shared_ptr<Frame> frame_current) {

    const cv::Mat &image_left_t1 = frame_current->image_left_;
    unsigned int frame_id = frame_current->getID();
    std::vector<cv::Point2f> points_left_t0;
    std::vector<cv::Point2f> points_left_t1;

    for(int i=0; i < frame_current->features_left_.size(); i++) {
        points_left_t0.push_back(frame_previous->features_left_[i]->getPoint2D());
        points_left_t1.push_back(frame_current->features_left_[i]->getPoint2D());
    }

    int radius = 2;
    cv::Mat vis;
    cv::cvtColor(image_left_t1, vis, cv::COLOR_GRAY2BGR, 3);

    for (int i = 0; i < points_left_t0.size(); i++)
    {
        cv::circle(vis, cv::Point(points_left_t0[i].x, points_left_t0[i].y), radius, CV_RGB(0,255,0));
    }
    for (int i = 0; i < points_left_t1.size(); i++)
    {
        cv::circle(vis, cv::Point(points_left_t1[i].x, points_left_t1[i].y), radius, CV_RGB(255,0,0));
    }
    for (int i = 0; i < points_left_t1.size(); i++)
    {
        cv::line(vis, points_left_t0[i], points_left_t1[i], CV_RGB(0,255,0));
    }
    cv::imshow("Optical flow ", vis );
    cv::waitKey(1);
}

void Viewer::displayTrajectory(const std::shared_ptr<Frame> frame_current) {

    if ( ! frame_previous) {
        frame_previous = frame_current;
        return;
    }

    unsigned int true_pose_id = frame_current->getID();
    Sophus::SE3d T_c_w = frame_current->getPose();
    Sophus::SE3d T_w_c = T_c_w.inverse();
    Eigen::Matrix3d rotation = T_c_w.rotationMatrix();
    Eigen::Vector3d translation = T_c_w.translation();
    int x = int(translation[0]) + 300;
    int y = int(translation[2]) + 100;
    cv::Mat overlay;
    trajectory_.copyTo(overlay);
    for (auto &mappoint : frame_current->getPoints3D()) {
        Eigen::Vector3d v3d(mappoint.x, mappoint.y, mappoint.z);
        //v3d = frame_current->getPose() * v3d;
        int mp_x = v3d.x() + 300;
        int mp_y = v3d.z() + 100;
        cv::circle(overlay, cv::Point(mp_x, mp_y) ,1, CV_RGB(126,126,126), 1);
    }
    cv::addWeighted(overlay, 0.05, trajectory_, 1 - 0.05, 0, trajectory_);
    cv::circle(trajectory_, cv::Point(x, y) ,1, CV_RGB(0,0,255), 2);

    // ground truth
    cv::Mat pose_gt = cv::Mat::zeros(1, 3, CV_64F);
    pose_gt.at<double>(0) = ground_truth_poses_[true_pose_id].at<double>(3);
    pose_gt.at<double>(1) = ground_truth_poses_[true_pose_id].at<double>(7);
    pose_gt.at<double>(2) = ground_truth_poses_[true_pose_id].at<double>(11);
    x = int(pose_gt.at<double>(0)) + 300;
    y = int(pose_gt.at<double>(2)) + 100;
    cv::circle(trajectory_, cv::Point(x, y) ,1, CV_RGB(0,255,0), 2);
    cv::imshow( "Trajectory", trajectory_ );
    cv::waitKey(1);
}

void Viewer::loadGroundTruthPoses() {

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
        ground_truth_poses_.push_back(pose);
    }
    file.close();
}

