#include <fstream>
#include <opencv2/opencv.hpp>
#include "sequence.h"

Sequence::Sequence(std::string sequence_path)
    : sequence_path_(sequence_path) {};

bool Sequence::Init() {
    LoadFrames(sequence_path_);
    return true;
}

void Sequence::LoadFrames(std::string filepath) {
    const std::string filename = filepath + "/times.txt";
    std::ifstream file;
    file.open(filename);
    if (!file) {
        throw std::runtime_error("Could not load timestamps file: " + filepath);
    }
    elements_.clear();
    int frame_index = 0;
    std::string  line;
    while (std::getline(file, line)) {
        std::istringstream timestamp_string(line);
        std::shared_ptr<StereoPair> pair = std::make_shared<StereoPair>();
        timestamp_string >> pair->timestamp_;
        pair->image_left_ = loadImageLeft(frame_index, sequence_path_);
        pair->image_right_ = loadImageRight(frame_index, sequence_path_);
        elements_.push_back(pair);
        ++frame_index;
    }

    file.close();
}

std::string Sequence::loadImageLeft(int frame_index, std::string filepath) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << frame_index;
    const std::string filename = filepath + "/image_0/" + ss.str() + ".png";
    return  filename;
}

std::string Sequence::loadImageRight(int frame_index, std::string filepath) {
    std::stringstream ss;
    ss << std::setfill('0') << std::setw(6) << frame_index;
    const std::string filename = filepath + "/image_1/" + ss.str() + ".png";
    return filename;
}


