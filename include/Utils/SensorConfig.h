#pragma once

#include <string>
#include <utility>
#include <vector>

struct CameraInfo {
    std::string name;
    std::string ip;
    int rFlag = -1;             // cv::RotateFlags, -1 = no rotation
    std::string calibFileName;
};

// Settings of the segmentation model (the "segmentation" block of sensors.json)
struct SegmentationConfig {
    std::string modelDir;       // Folder with model.onnx and config.json (id2label), empty = no model
    int inputSize = 512;        // Long side of the network input in pixels (the short side keeps the aspect ratio)
    int threads = 0;            // ONNX Runtime threads, 0 = all cores
    float minConfidence = 0.0f; // Pixels with a lower class probability are UNKNOWN
    std::string device = "auto"; // auto (GPU when available, else CPU), gpu (CPU when the GPU fails) or cpu
    std::vector<std::string> ignoreClasses;     // Classes left unlabelled (e.g. ceiling, floor)
    // Classes combined into one (name, member classes): their probabilities are added, e.g. plant/tree
    std::vector<std::pair<std::string, std::vector<std::string>>> mergeClasses;
    bool ignoreOtherClasses = false;            // Only the merged classes are labelled, all others stay unlabelled
};

// Settings of the occupancy map (the "occupancy_map" block of sensors.json): the lidar points seen from above
struct OccupancyMapConfig {
    float minHeight = 0.2f;     // Only points from this height above the ground plane [m] (above 0: the ground is left out)
    float maxHeight = 3.0f;     // up to this height [m] (e.g. leaves out a hall ceiling or overhanging branches)
    // Recorded occupancy grid (rosbag): a square of size x size metres around the lidar, in cells of cellSize
    float cellSize = 0.1f;
    float size = 40.0f;
};

struct SensorConfig {
    std::string lidarIp = "192.168.3.150";
    std::vector<CameraInfo> cameras;
    std::string robotHost = "192.168.1.20";
    int robotPort = 6379;
    std::string calibFileName = "config/calib.json";
    std::string pointCloudColor = "distance";  // Initial point cloud colouring: distance, rgb or segmentation
    SegmentationConfig segmentation;
    OccupancyMapConfig occupancyMap;
    std::string recordFormat = "folder";    // Initial recording format: folder, rosbag or both
    std::string sourceFile;     // File the config was read from, empty when the defaults are used

    // The rig defaults (used when no config file is found)
    static SensorConfig defaults();

    // Searches for the config file: the given path, ./config/sensors.json, then
    // config/sensors.json relative to the executable (build/bin -> repo root).
    static SensorConfig load(const std::string& path = "");
};

// Shared by the examples: the rig defaults with optional file overrides
const SensorConfig& sensorConfig();
