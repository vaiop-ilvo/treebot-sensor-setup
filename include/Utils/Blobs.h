#pragma once

#include <cstdint>
#include <vector>

#include <QVector3D>
#include <opencv2/core.hpp>

#include <Lidar/QtPoint.h>

// A group of connected points of one class, drawn as a circle around its points
struct Blob {
    QVector3D center;   // Centroid of the points (lidar frame) [m]
    QVector3D color;    // Class colour, RGB [0, 1]
    float radius;       // Covers 90% of the points [m]
    uint8_t label;
    uint32_t points;
};

struct BlobResult {
    std::vector<Blob> blobs;            // Blobs with at least minPoints points
    std::vector<int32_t> pointBlob;     // Per point: index in blobs, -1 when the point is in no (kept) blob
};

// Groups the labelled points in blobs: points of the same class closer than about linkDistance are connected
// (cells of linkDistance with their 26 neighbours). Blobs with fewer than minPoints points are dropped.
// Points with a label outside the palette (not seen) or with include[i] == 0 are left out (include may be empty).
BlobResult findBlobs(const std::vector<vaiop::QtPoint>& cloud, const std::vector<uint8_t>& labels,
                     const std::vector<cv::Vec3b>& palette, uint32_t minPoints, float linkDistance = 0.3f,
                     const std::vector<uint8_t>& include = {});
