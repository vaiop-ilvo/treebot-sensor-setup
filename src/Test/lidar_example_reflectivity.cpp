/**
 * Copyright (c) 2022, Ouster, Inc.
 * All rights reserved.
 */

#include <iostream>

#include <Lidar/Ouster.h>
#include <Utils/SensorConfig.h>

int main(int argc, char* argv[]) {

    std::unique_ptr<Ouster> ouster;
    try {
        ouster = std::make_unique<Ouster>(sensorConfig().lidarIp);
    } catch (const std::exception& e) {
        std::cerr << "Lidar not reachable at " << sensorConfig().lidarIp << ": " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    // 2. Initialize viewer
    cv::namedWindow("Lidar Viewer", cv::WINDOW_NORMAL);

    while (true)
    {
        if (ouster->scan(0.1)) {
            auto imgViz = ouster->getReflectivityImage();
            if (imgViz != nullptr) {
                cv::imshow("Lidar Viewer", *imgViz);
            }
        }

        // Wait for 1ms; check if 'q' was pressed
        char key = static_cast<char>(cv::waitKey(1));
        if (key == 'q' || key == 'Q') {
            break;
        }
    }

    return EXIT_SUCCESS;
}
