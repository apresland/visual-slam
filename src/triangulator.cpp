#include <memory>
#include "frame.h"
#include "triangulator.h"

void Triangulator::triangulate(std::shared_ptr<Frame> frame) {

    std::vector<cv::Point2f>  matches_2d_left = frame->get_points_left();
    std::vector<cv::Point2f>  matches_2d_right = frame->get_points_right();

    std::cout << "[INFO] Triangulator::triangulate - input points 2D { L"
              << matches_2d_left.size() << " : R"
              << matches_2d_right.size() << "}" << std::endl;

    cv::Mat proj_matrix_left = camera_left_->P();
    cv::Mat proj_matrix_right = camera_right_->P();
    cv::Mat points3D, points4D;
    cv::triangulatePoints(proj_matrix_left, proj_matrix_right, matches_2d_left, matches_2d_right, points4D);
    cv::convertPointsFromHomogeneous(points4D.t(), points3D);

    assert(points3D.rows == frame->features_left_.size());

    int num_predefined = 0;
    std::cout << "[INFO] Triangulator::triangulate - defining " << points3D.rows << " mappoints" << std::endl;
    for(int i=0; i< points3D.rows; ++i) {
        if ( frame->features_left_[i]->landmark_ ) num_predefined++;
        cv::Point3f p3d = *points3D.ptr<cv::Point3f>(i);
        std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>();
        map_point->point_3d_ = p3d;
        frame->features_left_[i]->landmark_ = map_point;
        frame->features_right_[i]->landmark_ = map_point;
    }

    std::cout << "[INFO] Triangulator::triangulate - " << num_predefined << " predefined mappoints "
              << frame->features_left_.size() - num_predefined << " new" << std::endl;
}