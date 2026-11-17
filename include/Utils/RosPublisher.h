#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Utils/Frame.h>
#include <Utils/SensorConfig.h>

// Publishes the live data on ROS 2 topics (the "ros" block of sensors.json), with the same messages and frames as
// the rosbag (BagWriter), so a navigation node can use the live topics or a played back bag alike:
//   /occupancy_map     nav_msgs/OccupancyGrid   -1 unknown, 0 free, 100 occupied; frame ground (x forward, y left,
//                                               the lidar above the origin), a new map every scan
//   /tf                tf2_msgs/TFMessage       ground -> lidar (ground plane of each scan)
//   /occupancy_map/classes        nav_msgs/OccupancyGrid   class id of the occupied cells (index in class_names),
//                                               -1 for free, unknown and unclassified cells; same geometry and stamp as
//                                               the /occupancy_map of that scan (segmentation colour mode, every
//                                               segmented scan)
//   /occupancy_map/class_names    std_msgs/String  JSON {"class id": "name"}, latched (transient local)
//   /robot/odom        nav_msgs/Odometry        odom -> base_link from the robot state (Redis): pose, measured
//                                               longitudinal [m/s] and angular [rad/s] velocity
// publish() is called by the Processor thread for every live frame. Without ROS 2 (built without WITH_ROSBAG)
// isAvailable() is false and start() fails.
class RosPublisher {
public:
    RosPublisher();
    ~RosPublisher();

    static bool isAvailable();

    // Creates the node and the publishers
    bool start(const RosConfig& config, std::string& error);
    void stop();
    bool isRunning() const;

    void publish(const ProcessedFrame& frame);
    // A segmented scan: its occupancy map with the class layer
    void publishClasses(uint64_t stampUs, const OccupancyMap& classMap);
    void setClassNames(const std::vector<std::string>& names);

private:
    struct Impl;
    std::unique_ptr<Impl> impl;
};
