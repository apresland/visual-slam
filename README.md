# Stereo Visual Odometry based SLAM
This repository contains a C++ implementation of __Simultaneous Localisation and Mapping (SLAM)__ with a stereo __Visual Odometry (VO)__ frontend to provide 
initial pose estimation. A demonstration is supplied using the KITTI dataset. 

## Overview
The VO frontend pipeline is responsible for online pose state-estimation. It is implemented with OpenCV and Eigen and consists of the following stages:
- __FAST Feature Detection__: Detect keypoints in greyscale images using the FAST algorithm.
- __Feature Bucketing__: Improve performance by bucketing features into grid cells and limiting the number in each cell.
- __Circular Feature Matching__: Improve robustness by keeping long lived features that can be stereo matched _and_ temporaly matched in two successive frames.
- __Lucas-Kandade Feature Tracking__: Improve performance by detecting new features only when necessary and using optical flow to track features between frames.

The backend is responsible for offline optimization of local and global maps. It is implmented using Ceres Solver and consists of the following stages:
- __Bundle Adjustment__: TBD
- __Pose Graph Optimization__: TBD
- __Loop Closure__: TBD

## Requirements
Docker is used to provide a ready made isolated development environment with all dependencies and the container can be used as a remote target in CLion.
If developing on the host machine at least C++14, CMake, OpenCV, Eigen, Sophus, Ceres Solver, ROS and Pangolin will be required. 
