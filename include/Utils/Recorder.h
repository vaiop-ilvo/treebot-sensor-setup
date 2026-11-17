#pragma once

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include <Utils/BagWriter.h>
#include <Utils/Frame.h>
#include <Utils/LidarCamera.h>
#include <Utils/SensorConfig.h>

// Records the frames in a rosbag (BagWriter): one bag per start() ... stop(), rosbag_YYYYMMDD_hhmmss in the folder.
// writeFrame is called by the Processor for every frame with record = true, writeSegmentation by the
// SegmentationWorker; both are thread safe and never block (the BagWriter converts on its own threads).
class Recorder {
public:
    Recorder(const SensorConfig& config, std::vector<CameraCalib> calibs);

    static bool isAvailable() { return BagWriter::isAvailable(); }
    // Names of the segmentation classes (class layer, label field); call before start()
    void setClassNames(std::vector<std::string> names);

    bool start(const std::string& folder, std::string& error);
    // Writes what is still queued and closes the bag
    void stop();
    bool isRecording() const;
    std::string uri() const;
    uint64_t rows() const;
    size_t pending() const;
    size_t dropped() const;

    // A row: scan, ground plane (/tf), occupancy map (OccupancyGrid; GridMap when withGridMap), robot state, the new
    // camera frames and /record/row
    void writeFrame(const Frame& frame, const GroundPlane& plane, std::shared_ptr<const OccupancyMap> occupancy, bool withGridMap);
    // The segmentation of a recorded frame: /lidar/points_segmented and the GridMap with the class layer
    void writeSegmentation(const Frame& frame, std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud,
                           const std::vector<uint8_t>& labels, std::shared_ptr<const OccupancyMap> classMap);

private:
    const SensorConfig config;
    const std::vector<CameraCalib> calibs;
    std::vector<std::string> classNames;
    BagWriter bag;
    std::atomic<bool> recording{false};
    std::atomic<uint64_t> rowCount{0};
    std::mutex mutex;                   // start / stop
};

// The configured Redis keys that the application does not know: redis:<key> columns of the CSV
std::vector<std::string> extraRobotKeys(const std::vector<RedisKey>& keys);
// Columns of the CSV log of the folder format (scripts/bag_to_folder.py, the Replay): timestamp, lidar_*, camN_*,
// robot_* and redis:<key> for the extra Redis keys
std::vector<std::string> folderLogHeader(int nCameras, const std::vector<std::string>& extraRobotKeys);
