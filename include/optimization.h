#ifndef VISUAL_SLAM_OPTIMIZER_H
#define VISUAL_SLAM_OPTIMIZER_H

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "frame.h"
#include "map.h"
#include "mappoint.h"

class Optimization{

public:
    Optimization() {}
    void compute(Map::KeyframesType keyframes,
                 Map::LandmarksType landmarks,
                 const cv::Mat &K);

public:
    static constexpr int pose_dims = 6;
    static constexpr int landmark_dims = 3;
};

#endif //VISUAL_SLAM_OPTIMIZER_H
