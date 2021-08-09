#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include<iomanip>
#include <memory>

#include "odometry.h"

int main(int argc, char **argv)
{
    std::string path_to_sequence = "/data/kitti/odometry/dataset/sequences/00/";
    std::shared_ptr<Odometry> odometry = std::make_shared<Odometry>(path_to_sequence);
    odometry->Init();
    odometry->Run();
    return 0;
}


