#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <Lidar/QtPoint.h>
#include <Utils/GroundPlane.h>
#include <Utils/MetaData.hpp>
#include <Utils/OccupancyMap.h>

// The data that flows through the pipeline (Acquisition -> Processor -> GUI / Recorder / SegmentationWorker).
// Everything is shared read-only after it is made: the point clouds and images are never changed in place.

// One camera in a row
struct CameraSample {
    std::string ip;
    bool open = false;              // The camera could be opened
    double fps = 0.0;               // Live status, for the camera tile
    uint64_t frameId = 0;
    bool newFrame = false;          // A frame for this row: close to the trigger and not used by the previous row
    cv::Mat image;                  // RGB, the frame of this row (newFrame)
    CameraMetaData meta;            // Of that frame (meta.valid = newFrame)
    cv::Mat colourImage;            // RGB, the frame used to colour the point cloud: close to the trigger and rotated as
                                    // calibrated (may also be the frame of the previous row); empty = not used
};

// One row: a lidar trigger with the camera frames and robot state that belong to it
struct Frame {
    uint64_t triggerTs = 0;         // Host time [us since epoch]: the lidar timestamp, or the soft trigger time
    OusterMetaData lidar;
    std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud;   // Coloured by distance; nullptr without a scan
    std::vector<CameraSample> cameras;
    RobotMetaData robot;
    std::string robotError;         // Why the robot state is not valid (empty while connecting)
    bool live = true;               // false for a frame of a replay
    bool record = false;            // Recorded in the rosbag
};

// A frame after processing: ground plane and occupancy map, computed once for the view and the recording
struct ProcessedFrame {
    std::shared_ptr<const Frame> frame;
    uint64_t tag = 0;               // Colour mode / live-replay generation: results with another tag are outdated
    GroundPlane plane;              // Invalid without a scan or without ground
    std::shared_ptr<const OccupancyMap> occupancy;              // Without classes; nullptr without a ground plane
    // Cloud to show: coloured by distance or with the cameras (RGB mode); nullptr in segmentation mode (the
    // segmented cloud follows from the SegmentationWorker)
    std::shared_ptr<const std::vector<vaiop::QtPoint>> displayCloud;
};
