# Stereo Visual Odometry

<img src="https://user-images.githubusercontent.com/5468707/146170834-990d25c9-dbb9-42ca-8c2a-3fb1a251f22e.gif" width=600>  <img src="https://user-images.githubusercontent.com/5468707/146187900-8fbb37cd-d156-41f1-9019-aeb0aff66a67.gif" height=180 width=180>

A stereo __Visual Odometry (VO)__ frontend to providing initial pose estimation and demonstrated using the KITTI dataset. Implemented in C++ and based on Robot Operating System (ROS) and OpenCV with Sophus and Eigen providing Lie Algebra and Linear Algebra. Docker eases dependency management and provides modularization with development in CLion configured to conect to a remote docker target. The solution publishes estimated poses and perception results to ROS topics that can be visualized using RViz running in a dedicated docker instance. Docker compose is used to run the multi-container application and compose file and docker files are provided.

## Algorithm Description

### 1. Image Acquisition
Capture a stereo image pair at time T and T+1 and process the images to compensate for lens distortion. Perform stereo rectification so that epipolar lines become parallel to horizontal. In KITTI dataset the input images are already corrected for lens distortion and stereo rectified.

### 2. Feature Detection
Generate features on the left camera image at time T using FAST (Features from Accelerated Segment Test) corner detector. FAST is computationally less expensive than other feature detectors like SIFT and SURF. Apply feature bucketing where the image is divided into non-overlapping rectangles and a constant number of feature points with maximal response values are then selected from each bucket. Bucketing has two benefits: i) Input features are well distributed throughout the image which results in higher accuracy in motion estimation. ii) The computation complexity of algorithm is reduced by the smaller sample of features.

### 3. Feature Matching(tracking)
Features generated in previous step are then searched in image at time T+1. More recent literature uses KLT (Kanade-Lucas-Tomasi) tracker for feature matching. Features from image at time T are tracked at time T+1 using a 15x15 search windows and 3 image pyramid level search. KLT tracker outputs the corresponding coordinates for each input feature and accuracy and error measure by which each feature was tracked. Feature points that are tracked with high error or lower accuracy are dropped from further computation.

### 3D Point Cloud Generation
### Inlier Detection
### Motion Estimation

The backend is responsible for offline optimization of local and global maps. It is implmented using Ceres Solver and consists of the following stages:
- __Bundle Adjustment__: TBD
- __Pose Graph Optimization__: TBD
- __Loop Closure__: TBD

## Requirements
Docker is used to provide a ready made isolated development environment with all dependencies and the container can be used as a remote target in CLion.
If developing on the host machine at least C++14, CMake, OpenCV, Eigen, Sophus, Ceres Solver, ROS and Pangolin will be required. 
