#pragma once

#include <string>
#include <vector>

struct CameraInfo {
    std::string name;
    std::string ip;
    int rFlag = -1;             // cv::RotateFlags, -1 = no rotation
    std::string calibFileName;
};

struct SensorConfig {
    std::string lidarIp = "192.168.3.150";
    std::vector<CameraInfo> cameras;
    std::string robotHost = "192.168.1.20";
    int robotPort = 6379;
    std::string calibFileName = "config/calib.json";
    std::string pointCloudColor = "distance";  // Initial point cloud colouring: distance, rgb or segmentation
    std::string sourceFile;     // File the config was read from, empty when the defaults are used

    // The rig defaults (used when no config file is found)
    static SensorConfig defaults();

    // Searches for the config file: the given path, ./config/sensors.json, then
    // config/sensors.json relative to the executable (build/bin -> repo root).
    static SensorConfig load(const std::string& path = "");
};

// Shared by the examples: the rig defaults with optional file overrides
const SensorConfig& sensorConfig();
