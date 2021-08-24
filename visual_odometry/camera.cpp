#include "camera.h"

cv::Point3d Camera::pixel2camera(const cv::Point2d &p, double depth) {
    return cv::Point3d(
            (p.x - cx_) * depth / fx_,
            (p.y - cy_) * depth / fy_,
            depth
    );
}

