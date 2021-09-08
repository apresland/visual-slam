#ifndef VISUAL_SLAM_OPTIMIZER_H
#define VISUAL_SLAM_OPTIMIZER_H

#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/core/block_solver.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

#include "frame.h"
#include "landmark.h"

class Optimization{

public:
    Optimization() {
        auto solver = new g2o::OptimizationAlgorithmLevenberg(
                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        optimizer_.setAlgorithm(solver);
        optimizer_.setVerbose(true);
    }

    void compute(std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes,
                 std::unordered_map<unsigned long, Landmark> landmarks,
                 const cv::Mat &K);

public:
    static constexpr int pose_dims = 6;
    static constexpr int landmark_dims = 3;
    typedef g2o::BlockSolver<g2o::BlockSolverTraits<pose_dims, landmark_dims>> BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    g2o::SparseOptimizer optimizer_;
};

#endif //VISUAL_SLAM_OPTIMIZER_H
