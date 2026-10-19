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

#include <opencv2/opencv.hpp>
#include <Utils/MetaData.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#include <Eigen/Dense>

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
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
    Eigen::MatrixXd eigenCloud; // N x 4 homogeneous points (only points with a return)
public:
    // Throws when the sensor cannot be reached
    explicit Ouster(const std::string& ip);
    ~Ouster() = default;

    // Fetches the next scan, waiting at most timeoutSec. Returns true for a new frame.
    bool scan(double timeoutSec = 0.0);
    void resetTimestamp();

    std::shared_ptr<std::vector<vaiop::QtPoint>> getPointCloud();
    std::shared_ptr<cv::Mat> getReflectivityImage();
    const Eigen::MatrixXd& getEigenCloud() const;
    const std::string& getIp() const;
    double getFps() const;
    uint64_t getTimeStamp() const;
    uint64_t getFrameId() const;
    uint64_t getScanCount() const;
    void getMetaData(OusterMetaData&) const;

    // Queues the last point cloud to be written to disk. Returns false when there is no cloud or the write queue is full.
    bool savePLY(const std::string& filename) const;
    // Reads a point cloud written by savePLY, coloured like getPointCloud. Returns nullptr when the file cannot be read.
    static std::shared_ptr<std::vector<vaiop::QtPoint>> loadPLY(const std::string& filename);
};
