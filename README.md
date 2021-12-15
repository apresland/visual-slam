# Stereo Visual Odometry
A stereo __Visual Odometry (VO)__ frontend to providing initial pose estimation and demonstrated using the KITTI dataset. Implemented in C++ and based on Robot Operating System (ROS) and OpenCV with Sophus and Eigen providing Lie Algebra and Linear Algebra. Docker eases dependency management and provides modularization with development in CLion configured to conect to a remote docker target. The solution publishes estimated poses and perception results to ROS topics that can be visualized using RViz running in a dedicated docker instance. Docker compose is used to run the multi-container application and compose file and docker files are provided.  

![kitti_optical_flow](https://user-images.githubusercontent.com/5468707/146170834-990d25c9-dbb9-42ca-8c2a-3fb1a251f22e.gif)

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
