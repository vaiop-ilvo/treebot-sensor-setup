#pragma once

#include <cstdint>
#include <vector>

#include <Utils/GroundPlane.h>
#include <Utils/SensorConfig.h>

// Occupancy grid on the ground plane, around the lidar, as recorded in the rosbag. Coordinates as in the occupancy
// map view: x forward, y left (ROS convention), in metres on the ground plane, the lidar above the origin.
// Cells are stored row by row from the corner (originX, originY): index = iy * cellsX + ix, as nav_msgs/OccupancyGrid.
struct OccupancyMap {
    static constexpr int8_t UNKNOWN = -1;      // No points
    static constexpr int8_t FREE = 0;          // Only points below minHeight (ground, low vegetation)
    static constexpr int8_t OCCUPIED = 100;    // Points between minHeight and maxHeight

    float cellSize = 0.1f;
    int cellsX = 0;
    int cellsY = 0;
    float originX = 0.0f;       // Ground position of the outer corner of cell (0, 0) [m]
    float originY = 0.0f;
    float minHeight = 0.0f;     // Height range of the occupied cells [m]
    float maxHeight = 0.0f;

    std::vector<int8_t> occupancy;
    std::vector<float> elevation;       // Highest point in the height range [m], NaN when none
    std::vector<uint8_t> classes;       // Most common class of the points in the height range, NO_LABEL when none
    bool hasClasses = false;            // Built from labelled points
};

// Builds the map from the points projected on the ground (projectToGround). The labels of the points give the class layer.
OccupancyMap buildOccupancyMap(const std::vector<BirdEyePoint>& points, const OccupancyMapConfig& config, bool labelled);
