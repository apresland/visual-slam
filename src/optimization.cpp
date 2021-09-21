#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/core/robust_kernel_impl.h>

#include "optimization.h"

void Optimization::compute(Map::KeyframesType keyframes,
                           Map::LandmarksType landmarks,
                           const cv::Mat &K) {

    std::cout << "[INFO] Optimization::compute -  " << keyframes.size() << " : " << landmarks.size() << std::endl;

    //typedef g2o::BlockSolver<g2o::BlockSolverTraits<pose_dims, landmark_dims>> BlockSolverType;
    typedef g2o::BlockSolver_6_3 BlockSolverType;
    typedef g2o::LinearSolverDense<BlockSolverType::PoseMatrixType> LinearSolverType;
    auto solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
    g2o::SparseOptimizer optimizer;
    optimizer.setAlgorithm(solver);
    optimizer.setVerbose(true);

    // camera intrinsics
    g2o::CameraParameters *camera = new g2o::CameraParameters(
            K.at<double>(0, 0), Eigen::Vector2d(K.at<double>(0, 2), K.at<double>(1, 2)), 0);
    camera->setId(0);
    optimizer.addParameter(camera);


    int vertex_id = 0;

    // pose
    std::unordered_map<unsigned long, g2o::VertexSE3Expmap*> pose_vertices;
    std::unordered_map<int, int> frameID_to_vertexID;
    for (auto &keyframe : keyframes) {
        auto kf = keyframe.second;
        std::cout << "Adding pose vertex for KF id " << kf->id_ << std::endl;
        g2o::VertexSE3Expmap *vertex_pose = new g2o::VertexSE3Expmap();  // camera vertex_pose
        vertex_pose->setId(vertex_id);

        vertex_pose->setEstimate(g2o::SE3Quat(
                kf->get_pose().rotationMatrix(),
                kf->get_pose().translation()));
        if (kf->id_ == 0)
            vertex_pose->setFixed(true);
        optimizer.addVertex(vertex_pose);
        pose_vertices.insert({kf->id_, vertex_pose});
        frameID_to_vertexID[kf->id_] = vertex_id;
        vertex_id++;
    }

    std::cout << "No. pose vertices: " << pose_vertices.size() << std::endl;

    double chi2_th = 5.991;  // robust kernel 阈值
    std::unordered_map<unsigned long, g2o::VertexSBAPointXYZ*> vertices_landmarks;
    std::unordered_map<int, int> landmarkID_to_vertexID;

    // landmarks
    for (auto &landmark : landmarks)
    {
        int landmark_id = landmark.second->id_;
        cv::Point3f p = landmark.second->point_3d_;
        g2o::VertexSBAPointXYZ *vertex = new g2o::VertexSBAPointXYZ;
        vertex->setEstimate(Eigen::Vector3d(p.x, p.y, p.z));
        vertex->setId(vertex_id);
        vertex->setMarginalized(true);
        vertices_landmarks[vertex_id] = vertex;
        landmarkID_to_vertexID[landmark_id] = vertex_id;
        optimizer.addVertex(vertex);
        vertex_id++;
    }

    // edges
    int edge_id = 0;
    for (auto &landmark : landmarks) {

        int landmark_id = landmark.second->id_;
        std::vector<std::weak_ptr<Feature>> observations = landmark.second->observations_;

        for (auto &observation : observations) {

            if ( observation.lock() == nullptr) continue;
            auto feature = observation.lock();
            auto frame = feature->frame_;


            unsigned long fvid = frameID_to_vertexID.at(frame->id_);
            unsigned long lmvid = landmarkID_to_vertexID.at(landmark.second->id_);

            g2o::EdgeProjectXYZ2UV *edge = new g2o::EdgeProjectXYZ2UV();
            edge->setId(edge_id);

            edge->setVertex(0, pose_vertices.at(fvid));
            edge->setVertex(1, vertices_landmarks.at(lmvid));
            edge->setMeasurement(Eigen::Matrix<double, 2, 1>(feature->point_2d_.x, feature->point_2d_.y));
            edge->setInformation(Eigen::Matrix<double, 2, 2>::Identity());
            edge->setParameterId(0,0);
            auto rk = new g2o::RobustKernelHuber();
            rk->setDelta(chi2_th);
            edge->setRobustKernel(rk);
            optimizer.addEdge(edge);
            edge_id++;
        }
    }

    std::cout << "No. landmark vertices: " << vertices_landmarks.size() << std::endl;
    std::cout << "No. edges: " << edge_id << std::endl;

    optimizer.initializeOptimization();
    optimizer.optimize(10);

}
