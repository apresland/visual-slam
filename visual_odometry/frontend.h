#ifndef VISUAL_SLAM_FRONTEND_H
#define VISUAL_SLAM_FRONTEND_H

#include <memory>
#include "frame.h"
#include "viewer.h"

class Frontend {
public:
    Frontend();
    void process(std::shared_ptr<Frame> frame);
    void SetViewer(std::shared_ptr<Viewer> viewer) { viewer_ = viewer; }

private:
    int detect_features();
    int match_features();

    cv::Ptr<cv::FeatureDetector> detector_;
    cv::Ptr<cv::DescriptorExtractor> descriptor_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;

    std::shared_ptr<Viewer> viewer_ = nullptr;
    std::shared_ptr<Frame> current_frame_ = nullptr;
    std::shared_ptr<Frame> previous_frame_ = nullptr;
};

#endif //VISUAL_SLAM_FRONTEND_HS
