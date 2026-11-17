#pragma once

#include <chrono>
#include <memory>

#include <Lidar/QtPoint.h>
#include <Lidar/curl_client.h>

#include <ouster/client.h>
#include <ouster/impl/build.h>
#include <ouster/lidar_scan.h>
#include <ouster/sensor_scan_source.h>
#include <ouster/types.h>
#include <ouster/xyzlut.h>

#include <Utils/MetaData.hpp>

class Ouster
{
private:
    std::string ip;
    std::unique_ptr<ouster::sdk::sensor::SensorScanSource> sourceClient;
    std::unique_ptr<CurlClient> httpClient;

    uint64_t frameId = 0;
    uint64_t scanCount = 0;     // Monotonic number of received scans (frame_id wraps around)
    double fps = 0.0;
    uint64_t ts = 0;
    uint64_t tsRef = 0;
    std::chrono::steady_clock::time_point tic;

    ouster::sdk::core::XYZLut xyz_lut_;
    ouster::sdk::core::SensorInfo info_;
    std::unique_ptr<ouster::sdk::core::LidarScan> scan_;
public:
    // Throws when the sensor cannot be reached
    explicit Ouster(const std::string& ip);
    ~Ouster() = default;

    // Fetches the next scan, waiting at most timeoutSec. Returns true for a new frame.
    bool scan(double timeoutSec = 0.0);
    void resetTimestamp();

    // The points of the last scan with a return, coloured by distance (a new cloud every call)
    std::shared_ptr<std::vector<vaiop::QtPoint>> getPointCloud();
    const std::string& getIp() const;
    double getFps() const;
    uint64_t getTimeStamp() const;
    uint64_t getFrameId() const;
    uint64_t getScanCount() const;
    void getMetaData(OusterMetaData&) const;
};
