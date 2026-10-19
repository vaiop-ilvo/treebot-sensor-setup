#pragma once
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <Lidar/QtPoint.h>
#include <opencv2/opencv.hpp>

// Colours a lidar point cloud with a camera image.
//   pLidar     N x 4 homogeneous lidar points (one point per row)
//   im         BGR image (CV_8UC3)
//   tLidarCam  4 x 4 lidar -> camera transform
//   K          3 x 3 camera intrinsics
// Points that are behind the camera or outside the image are coloured grey.
std::shared_ptr<std::vector<vaiop::QtPoint>> projectLidarCamera(const Eigen::MatrixXd& pLidar, const cv::Mat& im, const Eigen::Matrix4d& tLidarCam, const Eigen::Matrix3d& K);

// Loads the intrinsics and extrinsics of camName. Returns false (and identity matrices) when they cannot be read.
bool loadCalib(const std::string& calibFileName, const std::string& camName, Eigen::Matrix3d& K, Eigen::Matrix4d& tLidarCam);
