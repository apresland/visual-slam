#include <iostream>
#include <vector>
#include <sstream>
#include <fstream>
#include<iomanip>
#include <memory>

#include "system.h"

int main(int argc, char **argv)
{
    std::string path_to_sequence = "/data/kitti/odometry/dataset/sequences/00/";
    std::shared_ptr<System> odometry = std::make_shared<System>(path_to_sequence);
    odometry->Init();
    odometry->Run();
    return 0;
}


