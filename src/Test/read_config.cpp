#include <Utils/LidarCamera.h>
#include <Eigen/Dense>
#include <string>
#include <iostream>

int main(int argc, char const *argv[])
{
    // 0. Parse arguments
    std::string calibFileName = "";
    if (argc == 2) { // argc > 1 means there is at least one argument after the program name
        const char* firstArg = argv[1]; // argv[0] is the program name
        calibFileName = std::string(firstArg);
    } else {
        std::cerr << "Usage: " << argv[0] << " <calibFileName>\n";
        return 1;
    }

    std::string camName = "cam1";
    Eigen::Matrix4d extrinsics;
    Eigen::Matrix3d intrinsics;
    loadCalib(calibFileName, camName, intrinsics, extrinsics);

    std::cout << "intrinsics" << intrinsics;
    std::cout << "extrinsics" << extrinsics;

    return 0;
}
