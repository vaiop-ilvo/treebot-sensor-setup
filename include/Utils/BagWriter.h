#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <Lidar/QtPoint.h>
#include <Utils/GroundPlane.h>
#include <Utils/LidarCamera.h>
#include <Utils/MetaData.hpp>
#include <Utils/SensorConfig.h>

// Records the sensor data in a ROS 2 bag (rosbag2, MCAP storage), next to or instead of the folder recording.
// Messages are converted and written on background threads, so the write calls never block; when the disk cannot
// keep up, messages are dropped (dropped()). All calls are thread safe. Without ROS 2 (built without WITH_ROSBAG)
// isAvailable() is false and open() fails.
//
// Topics (frames: lidar = Ouster sensor frame, ground = lidar projected on the fitted ground plane, x forward,
// z up; camN = camera optical frame):
//   /lidar/points                  sensor_msgs/PointCloud2       x, y, z (lidar)
//   /lidar/points_segmented        sensor_msgs/PointCloud2       x, y, z, rgb (class colour), label (segmentation mode)
//   /camN/image/compressed         sensor_msgs/CompressedImage   PNG
//   /camN/camera_info              sensor_msgs/CameraInfo        from config/calib.json
//   /tf_static                     tf2_msgs/TFMessage            lidar -> camN (calibration)
//   /tf                            tf2_msgs/TFMessage            ground -> lidar (ground plane of each scan)
//   /occupancy_map                 nav_msgs/OccupancyGrid        -1 unknown, 0 free, 100 occupied (ground)
//   /occupancy_map/grid_map        grid_map_msgs/GridMap         layers occupancy, elevation, class (with grid_map_msgs)
//   /occupancy_map/class_names     std_msgs/String               JSON {"class id": "name"} of the class layer
//   /robot/odom                    nav_msgs/Odometry             odom -> base_link, from the robot state (Redis)
//   /robot/gnss                    sensor_msgs/NavSatFix
class BagWriter {
public:
    BagWriter();
    ~BagWriter();

    static bool isAvailable();
    // Built with grid_map_msgs: the occupancy map is also written as a GridMap with the class layer
    static bool hasGridMap();

    // Creates the bag in the folder <uri> (must not exist yet)
    bool open(const std::string& uri, std::string& error);
    // Writes the queued messages and closes the bag
    void close();
    bool isOpen() const;
    std::string uri() const;
    size_t pending() const;
    size_t dropped() const;

    // Once per bag: /tf_static and the camera_info written with every image
    void writeCalibration(uint64_t stampUs, const std::vector<CameraCalib>& calibs);
    // Once per bag: names of the class layer
    void writeClassNames(uint64_t stampUs, const std::vector<std::string>& names);

    // A lidar scan: /lidar/points, /tf (ground -> lidar), /occupancy_map and, when withGridMap, the GridMap without
    // class layer (in segmentation mode the GridMap comes with writeSegmentedScan instead)
    void writeScan(uint64_t stampUs, std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud, const GroundPlane& plane,
                   const OccupancyMapConfig& config, bool withGridMap);
    // A segmented scan (colour = class colour, labels = class per point): /lidar/points_segmented and the GridMap
    // with the class layer
    void writeSegmentedScan(uint64_t stampUs, std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud,
                            std::vector<uint8_t> labels, const GroundPlane& plane, const OccupancyMapConfig& config);
    // A camera frame (RGB): /camN/image/compressed and /camN/camera_info (camera = index, cam1 = 0)
    void writeImage(size_t camera, uint64_t stampUs, const cv::Mat& imageRGB);
    void writeRobot(uint64_t stampUs, const RobotMetaData& robot);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
