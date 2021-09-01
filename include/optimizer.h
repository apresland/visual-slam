#ifndef VISUAL_SLAM_OPTIMIZER_H
#define VISUAL_SLAM_OPTIMIZER_H

#include <g2o/types/slam3d/types_slam3d.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>

class Optimizer{

public:
    Optimizer() {
        optimizer_.setAlgorithm(solver_);
        optimizer_.setVerbose(true);
    }

    void initialize() {

    }
public:
    g2o::SparseOptimizer optimizer_;
    OptimizationAlgorithm*  solver_ = new g2o::OptimizationAlgorithmLevenberg(
                                g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));

    VertexSE3* prevVertex;
    Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Zero();

    vector<VertexSE3*> vertices;
    vector<EdgeSE3*> odometryEdges;
    vector<EdgeSE3*> edges;

};

#endif //VISUAL_SLAM_OPTIMIZER_H
