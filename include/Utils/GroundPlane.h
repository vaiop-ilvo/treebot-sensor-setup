#pragma once

#include <vector>

#include <Eigen/Dense>
#include <QVector3D>
#include <opencv2/core.hpp>

#include <Lidar/QtPoint.h>
#include <Utils/Blobs.h>

// Ground plane in the lidar frame: normal . p + d = 0, the normal points up (towards the sensor),
// so normal . p + d is the height of p above the ground.
struct GroundPlane {
    bool valid = false;
    Eigen::Vector3f normal = Eigen::Vector3f::UnitZ();
    float d = 0.0f;
    size_t inliers = 0;

    float height(const Eigen::Vector3f& p) const { return normal.dot(p) + d; }
    // Height of the lidar above the ground [m]
    float sensorHeight() const { return d; }
    // Angle between the lidar z axis and the ground normal [deg]
    float tiltDeg() const;
};

// Fits the ground plane with RANSAC: the plane with the most points among the near-horizontal planes
// (normal within MAX_TILT of the lidar z axis) below the sensor, refined by least squares on its inliers.
// Walls (not horizontal) and the ceiling (above the sensor) are never chosen. With a previous plane the
// result is smoothed over time, unless the ground changed a lot. Returns an invalid plane when no ground is found.
GroundPlane fitGroundPlane(const std::vector<vaiop::QtPoint>& cloud, const GroundPlane* previous = nullptr);

// A point in the bird's-eye view: coordinates on the ground plane, with the lidar above the origin
struct BirdEyePoint {
    float forward;      // Along the lidar x axis projected on the ground [m]
    float left;         // Perpendicular to forward, to the left (lidar y side) [m]
    float height;       // Above the ground [m]
    QVector3D color;    // RGB [0, 1]
};

// Projects the cloud on the ground plane
std::vector<BirdEyePoint> projectToGround(const std::vector<vaiop::QtPoint>& cloud, const GroundPlane& plane);

// A blob in the bird's-eye view: its centre projected on the ground plane
struct BirdEyeBlob {
    float forward;
    float left;
    float height;       // Of the centre above the ground [m]
    float radius;       // [m]
    QVector3D color;
};
std::vector<BirdEyeBlob> projectToGround(const std::vector<Blob>& blobs, const GroundPlane& plane);

// View of the bird's-eye image
struct BirdEyeView {
    cv::Size size{800, 800};       // Image size [px]
    float metersPerPixel = 0.05f;
    cv::Point2f center{0, 0};      // Ground position (forward, left) at the image centre [m]
    float minHeight = -0.3f;       // Points outside [minHeight, maxHeight] above the ground are left out
    float maxHeight = 4.0f;        // (e.g. a ceiling would cover everything)
    float groundHeight = 0.15f;    // Points up to this height are ground: drawn dimmer, under the objects
    int pointSize = 2;             // [px]

    // Image position of a ground position; forward is up, left is left
    cv::Point2f toPixel(float forward, float left) const {
        return {size.width * 0.5f - (left - center.y) / metersPerPixel, size.height * 0.5f - (forward - center.x) / metersPerPixel};
    }
};

// Renders the points top-down (RGB, CV_8UC3, background colour bg). Where points overlap, the highest is shown,
// so objects are drawn on top of the ground below them.
cv::Mat renderBirdEye(const std::vector<BirdEyePoint>& points, const BirdEyeView& view, const cv::Vec3b& bg);
