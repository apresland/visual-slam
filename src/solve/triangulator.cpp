#include <memory>
#include <sensor/frame.h>
#include <solve/triangulator.h>

void Triangulator::triangulate(Context &context, bool is_initialize) {

    std::shared_ptr<Frame> &frame = context.frame_previous_;
    std::vector<cv::Point2f>  matches_2d_left = frame->getPointsLeft();
    std::vector<cv::Point2f>  matches_2d_right = frame->getPointsRight();

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
        cv::Point3f point_3d = *points3D.ptr<cv::Point3f>(i);
        if ( frame->features_left_[i]->getLandmark() )
        {
            frame->features_left_[i]->getLandmark()->setPoint3D(point_3d);
            frame->features_left_[i]->getLandmark()->addObservation(frame->features_left_[i]);
            num_predefined++;
        } else {
            std::shared_ptr<MapPoint> map_point = std::make_shared<MapPoint>();
            map_point->setID(landmark_id_);
            frame->features_left_[i]->setLandmark(map_point);
            map_->insertLandmark(map_point);
            map_point->setPoint3D(point_3d);
            map_point->addObservation(frame->features_left_[i]);
            landmark_id_++;
        }
    }

    std::cout << "[INFO] Triangulator::triangulate - " << num_predefined << " predefined mappoints "
              << frame->features_left_.size() - num_predefined << " new" << std::endl;
}