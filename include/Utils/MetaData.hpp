#pragma once

#include <cstdint>
#include <string>

struct RobotMetaData {
    bool valid = false; // True when the data was read from Redis
    double x = 0.0; // X coordinate
    double y = 0.0; // Y coordinate
    double z = 0.0; // Z coordinate
    double roll = 0.0; // Roll angle
    double pitch = 0.0; // Pitch angle
    double yaw = 0.0; // Yaw angle
    double lat = 0.0; // Latitude
    double lon = 0.0; // Longitude
    int gpsFix = 0; // GPS fix status
    double gpsLngSpeed = 0.0; // GPS velocity
    double odomLngSpeed = 0.0; // Odometer velocity
    double odomAngSpeed = 0.0; // Odometer angular velocity
};


struct OusterMetaData {
    bool valid = false; // True when a scan was received
    uint64_t frameId = 0;
    uint64_t ts = 0; // Host time in microseconds since epoch
    double fps = 0.0;
    std::string ip;
    std::string pcPath; // Path to saved point cloud file
};

struct CameraMetaData {
    bool valid = false; // True when a new frame was received for this row
    uint64_t frameId = 0;
    uint64_t ts = 0; // Host time in microseconds since epoch
    double fps = 0.0;
    std::string ip;
    std::string imPath; // Path to saved image file
};
