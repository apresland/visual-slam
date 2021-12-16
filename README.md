# Stereo Visual Odometry
<img src="https://user-images.githubusercontent.com/5468707/146170834-990d25c9-dbb9-42ca-8c2a-3fb1a251f22e.gif" width=75%>  <img src="https://user-images.githubusercontent.com/5468707/146187900-8fbb37cd-d156-41f1-9019-aeb0aff66a67.gif" height=217 width=217>

Visual odometry (VO) is the process of estimating the egomotion of an agent (e.g., vehicle, human, and robot) using only the input of a single or multiple cameras attached to it. Application domains include robotics, wearable computing, augmented reality, and automotive. The advantage of VO with respect to wheel odometry is that VO is not affected by wheel slip in uneven terrain or other adverse conditions. It has been demonstrated that compared to wheel odometry, VO provides more accurate trajectory estimates, with relative position error ranging from 0.1 to 2%.

## Project Desription
This project provides a complete __Stereo Visual Odometry (VO)__ frontend providing pose estimation and demonstrated using the KITTI dataset. The solution publishes estimated poses and perception results to ROS topics that can be visualized using RViz running in a dedicated docker instance. The project dependencies are summarised as follows:

* Implemented in __C++(17)__ using __CMake__.
* __OpenCV__ for FAST feature detection, KLT optical-flow, PnP and RANSAC.
* __Robot Operating System (ROS)__ as middleware.
* __Sophus__ for Lie Algebra and __Eigen__ for Linear Algebra.
* __Ceres Solver__ for local pose-graph optimization.
* __Rviz__ to visualize perception and estimated poses.
* __Docker__ to ease dependency management and provide modular services.
* __Docker-Compose__ to run the multi-container application and provide a dedicated network.
* __CLion__ configured to conect to a remote docker target.

## Project structure
```
.
├── CMakeLists.txt
├── docker_build.sh
├── docker_up.sh
├── docker_down.sh
├── docker-compose.yaml
├── Dockerfile_odometry
├── Dockerfile_visualizer
├── odometry.env
├── odometry.rviz
├── src
│   ├── backend.cpp
│   ├── backend.h
│   ├── CMakeLists.txt
│   ├── context.h
│   ├── frontend.cpp
│   ├── frontend.h
│   ├── map
│   │   ├── CMakeLists.txt
│   │   ├── map.cpp
│   │   └── map.h
│   ├── optimize
│   │   ├── CMakeLists.txt
│   │   ├── optimization.cpp
│   │   └── optimization.h
│   ├── package.xml
│   ├── sensor
│   │   ├── camera.cpp
│   │   ├── camera.h
│   │   ├── CMakeLists.txt
│   │   ├── feature.h
│   │   ├── frame.h
│   │   ├── mappoint.cpp
│   │   └── mappoint.h
│   ├── sequence.cpp
│   ├── sequence.h
│   ├── solve
│   │   ├── CMakeLists.txt
│   │   ├── detector.cpp
│   │   ├── detector.h
│   │   ├── estimation.cpp
│   │   ├── estimation.h
│   │   ├── matcher.cpp
│   │   ├── matcher.h
│   │   ├── tracker.cpp
│   │   ├── tracker.h
│   │   ├── triangulator.cpp
│   │   └── triangulator.h
│   ├── system.cpp
│   ├── system.h
│   ├── vizualization.cpp
│   └── vizualization.h
```

## Problem Formulation
<img src="https://user-images.githubusercontent.com/5468707/146366133-da7a62ba-8f5f-4970-9551-e88ca9b35fc8.png" width=600)>
The vehicle is in motion and taking images with a rigidly attached camera system at discrete time instants <i>k</i>. This results in a left and a right image
at every time instant, denoted by 
<i>I<sub>l,0:n</sub></i> = {<i>I<sub>l,0</i>, ... , <i>I<sub>l,n</i>} and <i>I<sub>r,0:n</sub></i> = {<i>I<sub>r,0</sub></i>, ... , <i>I<sub>r,n</sub></i>}
as shown in the illustration.

Two camera positions at adjacent time instants <i>k<sub>1</sub></i> and <i>k</i> are related by the rigid body transformation <i>T<sub>k,k-1</sub></i> of the following form:
<p align="center">  
  | &nbsp;<i>R<sub>k,k-1</sub></i> &nbsp;  <i>t<sub>k,k1</sub></i> &nbsp;| <br>
  | &nbsp;&nbsp;&nbsp;&nbsp;0 &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; 1 &nbsp;|
</p>
  
where <i>R<sub>k,k1</sub></i> an element of SO(3) is the rotation matrix, and <i>t<sub>k,k-1</sub></i> is the translation vector. 
The set <i>T<sub>1:n</sub></i> = {<i>T<sub>1,0</sub></i>, ... , <i>T<sub>n,n-1</sub></i>} contains all subsequent motions. The set of camera poses <i>C<sub>0:n</sub></i> = {<i>C<sub>0</sub></i>, ... , <i>C<sub>n</sub></i>} contains the transformations of the camera
with respect to the initial coordinate frame at <i>k</i> = 0. The current pose <i>C<sub>n</sub></i> can be computed by concatenating all the
transformations <i>T<sub>k</sub></i> (<i>k</i> = 1 ... <i>n</i>), and, therefore, <i>C<sub>n</sub></i> = <i>C<sub>n-1</sub>T<sub>n</sub></i>, with <i>C<sub>0</sub></i> being the camera pose at the instant <i>k</i> = 0. 
  
The task of VO is to compute the relative transformations <i>T<sub>k</sub></i> from the images <i>I<sub>k</sub></i> and <i>I<sub>k-1</sub></i> and then to concatenate the transformations to recover the full trajectory <i>C<sub>0:n</sub></i> of the camera. This means that VO recovers the path incrementally, pose after pose. 
  
## Algorithm Description
The VO algorithm with 3-D-to-2-D correspondences is summarized as follows.

> Algorithm: Visual Odometry from 3-D-to-2-D Correspondences.  
> 1. Do only once:  
1.1      &nbsp;&nbsp;&nbsp; Capture two frames <i>I<sub>k-2</sub></i>, <i>I<sub>k-1</sub></i>  
1.2      &nbsp;&nbsp;&nbsp; Detect and match features between frames  
1.3      &nbsp;&nbsp;&nbsp; Triangulate features from <i>I<sub>k-2</sub></i>, <i>I<sub>k-1</sub></i>  
>
> 2. Do at each iteration:  
2.1    &nbsp;&nbsp;&nbsp; Capture new frame I<sub>k</sub>  
2.2    &nbsp;&nbsp;&nbsp; Detect features and match with previous frame I<sub>k-1</sub>  
2.3    &nbsp;&nbsp;&nbsp; Compute camera pose (PnP) from 3-D-to-2-D matches  
2.4    &nbsp;&nbsp;&nbsp; Triangulate all new feture matches between I<sub>k</sub> and I<sub>k-1</sub>  
2.5   &nbsp;&nbsp;&nbsp; Go to 2.1

### 1. Image Capture
Capture a stereo image pair at time <i>k</i> and <i>k-1</i> and process the images to compensate for lens distortion. Perform stereo rectification so that epipolar lines become parallel to horizontal. In KITTI dataset the input images are already corrected for lens distortion and stereo rectified.

### 2. Feature Detection
Generate features on the left camera image <i>I<sub>l,k-1</sub></i> using FAST (Features from Accelerated Segment Test) corner detector. FAST is computationally less expensive than other feature detectors like SIFT and SURF. Apply feature bucketing where the image is divided into non-overlapping rectangles and a constant number of feature points with maximal response values are then selected from each bucket. Bucketing has two benefits: i) Input features are well distributed throughout the image which results in higher accuracy in motion estimation. ii) The computation complexity of algorithm is reduced by the smaller sample of features.

### 3. Feature Matching (Tracking)
Egomotion requires features to be matched between the left and right images at time <i>k-1</i> and with the left image at time <i>k</i>. Therefore we match in a ’circle’ starting from features detected in the current left image, the best match is found in the previous left image, next in the previous right image, the current right image and last in the current left image again. A circle match gets accepted, if the last feature coincides with the first feature. We use KLT (Kanade-Lucas-Tomasi) method for matching. Features from image at time <i>k-1</i> are tracked at time <i>k</i> using a 15x15 search windows and 3 image pyramid level search. KLT tracker outputs the corresponding coordinates for each input feature and accuracy and error measure by which each feature was tracked. Feature points that are tracked with high error or lower accuracy are dropped from further computation.

### Motion Estimation
Motion estimation is the computation of the camera motion between the current image and the previous image. Concatenating these single movements recovers the full trajectory of the vehicle. The transformation _T<sub>k_</sub> between two images _I<sub>k-1</sub>_ and _I<sub>k</sub>_ can be computed from two sets of corresponding features _f<sub>k-1</sub>_ and _f<sub>k</sub>_ at time instants _k-1_ and _k_ respectively. In general feature correspondences can be specified in 2-D or 3-D but we choose 3D-to-2D correspondences where _f<sub>k-1</sub>_ are specified in 3-D and _f<sub>k</sub>_ are their corresponding 2-D reprojections on the image _I<sub>k</sub>_. The general formulation in this case is to find _T<sub>k</sub>_ that minimizes the sum of the image reprojection error

<p align="center">
  <i>argmin</i>[ <i>sum</i>( || <i>p<sub>k</sub></i> - <i>p<sub>k-1</sub></i> ||<sup>2</sup> ) ]
</p>

where _p<sub>k-1</sub>_ is the reprojection of the 3-D point _X<sub>k-1</sub>_ into image _I<sub>k</sub>_ according to the transformation _T<sub>k</sub>_. This problem is known as perspective from n points (PnP).

### Triangulation and Keyframe Selection
The previous motion estimation methods require triangulation of 3-D points (structure) from 2-D image correspondences. Triangulated 3-D points are determined by intersecting back-projected rays from 2-D image correspondences of at two image frames. In perfect conditions, these rays would intersect in a single 3-D point. However, because of image noise, camera model and calibration errors, and feature matching uncertainty, they never intersect. Therefore, the point at a minimal distance, in the least-squares sense, from all intersecting rays can be taken as an estimate of the 3-D point position.
