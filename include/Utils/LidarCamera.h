#pragma once
#include <string>

#include <Eigen/Dense>
#include <opencv2/opencv.hpp>

// Calibration of one camera with respect to the lidar, as stored in config/calib.json
struct CameraCalib {
    std::string name;
    bool valid = false;
    Eigen::Matrix3d K = Eigen::Matrix3d::Identity();            // Camera matrix
    cv::Mat distCoeffs = cv::Mat::zeros(1, 5, CV_64F);          // OpenCV distortion coefficients
    Eigen::Matrix4d tLidarCam = Eigen::Matrix4d::Identity();    // Lidar -> camera: X_cam = T * X_lidar
    cv::Size imageSize;                                         // Image the calibration is valid for, (0, 0) when unknown
    int rotationDeg = -1;                                       // Camera rotation during calibration, -1 when unknown

    // Extent of the image on the normalised image plane (x/z, y/z), used to skip points outside the
    // field of view before projecting: the distortion polynomial folds far-away points back into the image.
    double xMin = -1e9, xMax = 1e9, yMin = -1e9, yMax = 1e9;
    // Squared radius on the normalised image plane up to which the radial distortion is monotonic
    double maxRadius2 = 1e18;
};

// Loads the calibration of camName. Returns an invalid CameraCalib when it cannot be read.
CameraCalib loadCameraCalib(const std::string& calibFileName, const std::string& camName);

// Loads the intrinsics and extrinsics of camName. Returns false (and identity matrices) when they cannot be read.
bool loadCalib(const std::string& calibFileName, const std::string& camName, Eigen::Matrix3d& K, Eigen::Matrix4d& tLidarCam);
