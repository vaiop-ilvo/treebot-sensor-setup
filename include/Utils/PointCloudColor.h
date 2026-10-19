#pragma once

#include <string>
#include <vector>

#include <QVector3D>
#include <opencv2/core.hpp>

#include <Lidar/QtPoint.h>
#include <Utils/LidarCamera.h>

// How the point cloud is coloured in the viewer
enum class ColorMode {
    Distance,       // Turbo colormap on the range to the lidar
    Rgb,            // Colour of the camera pixel the point projects onto
    Segmentation,   // Class colour of a segmentation model on the camera images
};

// "distance", "rgb", "segmentation" (case insensitive); Distance for anything else
ColorMode colorModeFromString(const std::string& text);
const char* toString(ColorMode mode);

// Turbo colormap (polynomial approximation by Google), t in [0, 1]
QVector3D turbo(float t);

// Colours every point by its range to the lidar
void colorByDistance(std::vector<vaiop::QtPoint>& cloud);

// Colours lidar points with the camera images.
// A point gets the colour of the camera that sees it closest to its image centre. Points that no
// camera sees, or that are hidden behind a nearer object in the camera view (the cameras sit
// 20-35 cm from the lidar, so they do not see exactly the same surfaces), are coloured grey.
class CameraColorizer {
public:
    CameraColorizer() = default;
    explicit CameraColorizer(std::vector<CameraCalib> calibs);

    const std::vector<CameraCalib>& calibs() const { return calibrations; }
    // Number of cameras with a valid calibration
    size_t validCount() const;

    // Where a point is seen: the camera that sees it closest to its image centre, and the pixel
    struct PointView {
        int camera = -1;    // -1: no camera sees the point
        cv::Point pixel;
    };
    // imageSizes[i]: size of the image of camera i, in the order of the calibrations; empty when there is no image.
    // Cameras whose image size differs from the calibration are skipped.
    std::vector<PointView> visibility(const std::vector<vaiop::QtPoint>& cloud, const std::vector<cv::Size>& imageSizes) const;

    // imagesRGB[i]: RGB image (CV_8UC3) of camera i, in the order of the calibrations; empty when missing.
    // Images with another size than the calibration are skipped. Returns the number of coloured points.
    size_t colorize(std::vector<vaiop::QtPoint>& cloud, const std::vector<cv::Mat>& imagesRGB) const;

    // Same with class maps (CV_8U, class id per pixel, e.g. from Segmenter) instead of images: a point gets the
    // palette colour (RGB) of its class. labels (optional) receives the class per point, UNSEEN_LABEL when no
    // camera sees the point; pixels with a class outside the palette count as not seen.
    static constexpr uint8_t UNSEEN_LABEL = 255;
    size_t colorizeLabels(std::vector<vaiop::QtPoint>& cloud, const std::vector<cv::Mat>& labelMaps,
                          const std::vector<cv::Vec3b>& palette, std::vector<uint8_t>* labels = nullptr) const;

private:
    std::vector<CameraCalib> calibrations;
};
