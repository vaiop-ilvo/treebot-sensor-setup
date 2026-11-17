#pragma once

#include <memory>
#include <string>
#include <vector>

#include <Lidar/QtPoint.h>

// Point clouds as binary PLY files (little endian, float x y z, optionally uchar red green blue), the format of
// the lidar/NNNNNN.ply files of a recording folder. The reader also reads the files written by PCL (which add a
// "camera" element after the vertices) and any vertex element with x, y, z of a numeric type.

// Writes the points; with withColor also their colour (QtPoint::color, RGB in [0, 1]). Returns false on failure.
bool writePly(const std::string& fileName, const std::vector<vaiop::QtPoint>& cloud, bool withColor = false);

// Reads the points (and the colours, when the file has red/green/blue). Returns nullptr when the file cannot be read.
std::shared_ptr<std::vector<vaiop::QtPoint>> readPly(const std::string& fileName);
