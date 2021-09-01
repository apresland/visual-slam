#ifndef VISUAL_SLAM_OBSERVATION_H
#define VISUAL_SLAM_OBSERVATION_H

struct Observation {
public:
    Observation(int frame_id, int feature_id)
        : frame_id_(frame_id), feature_id_(feature_id) {}
public:
    int frame_id_;
    int feature_id_;
};

#endif //VISUAL_SLAM_OBSERVATION_H
