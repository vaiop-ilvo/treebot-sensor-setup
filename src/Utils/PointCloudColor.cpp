#include <Utils/PointCloudColor.h>

#include <algorithm>
#include <cmath>
#include <limits>

#include <QString>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>

namespace {
    // Range used for the distance colormap [m]
    constexpr float COLOR_RANGE_MIN = 0.0f;
    constexpr float COLOR_RANGE_MAX = 15.0f;

    // Colour of points no camera sees
    const QVector3D UNSEEN_COLOR(0.3f, 0.3f, 0.3f);

    // Points closer than this to a camera are not coloured [m]
    constexpr float MIN_DEPTH = 0.2f;
    // Occlusion test: depth buffer cells of DEPTH_CELL x DEPTH_CELL pixels. A point is visible when it is
    // at most max(OCCLUSION_MIN, OCCLUSION_REL * depth) behind the nearest point around its cell.
    constexpr int DEPTH_CELL = 4;
    constexpr float OCCLUSION_MIN = 0.15f;
    constexpr float OCCLUSION_REL = 0.05f;
}

ColorMode colorModeFromString(const std::string& text)
{
    const QString mode = QString::fromStdString(text).trimmed().toLower();
    if (mode == "rgb") return ColorMode::Rgb;
    if (mode == "segmentation") return ColorMode::Segmentation;
    return ColorMode::Distance;
}

const char* toString(ColorMode mode)
{
    switch (mode) {
        case ColorMode::Rgb: return "rgb";
        case ColorMode::Segmentation: return "segmentation";
        default: return "distance";
    }
}

QVector3D turbo(float t)
{
    t = std::clamp(t, 0.0f, 1.0f);
    const float r = 0.13572138f + t * (4.61539260f + t * (-42.66032258f + t * (132.13108234f + t * (-152.94239396f + t * 59.28637943f))));
    const float g = 0.09140261f + t * (2.19418839f + t * (4.84296658f + t * (-14.18503333f + t * (4.27729857f + t * 2.82956604f))));
    const float b = 0.10667330f + t * (12.64194608f + t * (-60.58204836f + t * (110.36276771f + t * (-89.90310912f + t * 27.34824973f))));
    return QVector3D(std::clamp(r, 0.0f, 1.0f), std::clamp(g, 0.0f, 1.0f), std::clamp(b, 0.0f, 1.0f));
}

void colorByDistance(std::vector<vaiop::QtPoint>& cloud)
{
    for (vaiop::QtPoint& p : cloud) {
        p.color = turbo((p.position.length() - COLOR_RANGE_MIN) / (COLOR_RANGE_MAX - COLOR_RANGE_MIN));
    }
}

CameraColorizer::CameraColorizer(std::vector<CameraCalib> calibs)
    : calibrations(std::move(calibs))
{
}

size_t CameraColorizer::validCount() const
{
    return std::count_if(calibrations.begin(), calibrations.end(), [](const CameraCalib& c) { return c.valid; });
}

std::vector<CameraColorizer::PointView> CameraColorizer::visibility(const std::vector<vaiop::QtPoint>& cloud,
                                                                   const std::vector<cv::Size>& imageSizes) const
{
    const size_t n = cloud.size();
    std::vector<PointView> views(n);
    // Per point: squared distance to the image centre (normalised plane) of the camera that sees it
    std::vector<float> bestScore(n, std::numeric_limits<float>::infinity());

    for (size_t c = 0; c < calibrations.size() && c < imageSizes.size(); ++c) {
        const CameraCalib& calib = calibrations[c];
        const cv::Size& imageSize = imageSizes[c];
        if (!calib.valid || imageSize.area() == 0) {
            continue;
        }
        if (calib.imageSize.area() > 0 && imageSize != calib.imageSize) {
            continue;   // Other rotation or resolution than calibrated
        }

        // 1. Transform to the camera frame and keep the points inside the field of view
        const Eigen::Matrix3f R = calib.tLidarCam.block<3, 3>(0, 0).cast<float>();
        const Eigen::Vector3f t = calib.tLidarCam.block<3, 1>(0, 3).cast<float>();
        std::vector<cv::Point3f> camPoints;
        std::vector<size_t> index;
        camPoints.reserve(n / 4);
        index.reserve(n / 4);
        for (size_t i = 0; i < n; ++i) {
            const QVector3D& q = cloud[i].position;
            const Eigen::Vector3f p = R * Eigen::Vector3f(q.x(), q.y(), q.z()) + t;
            if (p.z() < MIN_DEPTH) {
                continue;
            }
            const float x = p.x() / p.z();
            const float y = p.y() / p.z();
            if (x < calib.xMin || x > calib.xMax || y < calib.yMin || y > calib.yMax || x * x + y * y > calib.maxRadius2) {
                continue;
            }
            camPoints.emplace_back(p.x(), p.y(), p.z());
            index.push_back(i);
        }
        if (camPoints.empty()) {
            continue;
        }

        // 2. Project with the lens distortion
        cv::Mat K;
        cv::eigen2cv(calib.K, K);
        std::vector<cv::Point2f> pixels;
        cv::projectPoints(camPoints, cv::Vec3d(0, 0, 0), cv::Vec3d(0, 0, 0), K, calib.distCoeffs, pixels);

        // 3. Depth buffer for the occlusion test: nearest point per cell, then the minimum over 3 x 3 cells
        //    (the lidar is sparser than the image, so a single cell would let the background shine through)
        const int cols = (imageSize.width + DEPTH_CELL - 1) / DEPTH_CELL;
        const int rows = (imageSize.height + DEPTH_CELL - 1) / DEPTH_CELL;
        cv::Mat depth(rows, cols, CV_32F, cv::Scalar(std::numeric_limits<float>::infinity()));
        auto inImage = [&imageSize](const cv::Point2f& px) {
            return px.x >= 0.0f && px.y >= 0.0f && px.x < imageSize.width - 0.5f && px.y < imageSize.height - 0.5f;
        };
        for (size_t k = 0; k < pixels.size(); ++k) {
            if (!inImage(pixels[k])) {
                continue;
            }
            float& d = depth.at<float>(static_cast<int>(pixels[k].y) / DEPTH_CELL, static_cast<int>(pixels[k].x) / DEPTH_CELL);
            d = std::min(d, camPoints[k].z);
        }
        cv::erode(depth, depth, cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));

        // 4. Keep the visible points, with the camera that sees them closest to its image centre
        for (size_t k = 0; k < pixels.size(); ++k) {
            const cv::Point2f& px = pixels[k];
            if (!inImage(px)) {
                continue;
            }
            const float z = camPoints[k].z;
            const float nearest = depth.at<float>(static_cast<int>(px.y) / DEPTH_CELL, static_cast<int>(px.x) / DEPTH_CELL);
            if (z > nearest + std::max(OCCLUSION_MIN, OCCLUSION_REL * nearest)) {
                continue;
            }
            const float x = camPoints[k].x / z;
            const float y = camPoints[k].y / z;
            const float score = x * x + y * y;
            const size_t i = index[k];
            if (score >= bestScore[i]) {
                continue;
            }
            bestScore[i] = score;
            views[i].camera = static_cast<int>(c);
            views[i].pixel = cv::Point(static_cast<int>(px.x + 0.5f), static_cast<int>(px.y + 0.5f));
        }
    }
    return views;
}

size_t CameraColorizer::colorize(std::vector<vaiop::QtPoint>& cloud, const std::vector<cv::Mat>& imagesRGB) const
{
    std::vector<cv::Size> sizes(imagesRGB.size());
    for (size_t c = 0; c < imagesRGB.size(); ++c) {
        if (!imagesRGB[c].empty() && imagesRGB[c].type() == CV_8UC3) {
            sizes[c] = imagesRGB[c].size();
        }
    }
    const std::vector<PointView> views = visibility(cloud, sizes);

    size_t nColored = 0;
    for (size_t i = 0; i < cloud.size(); ++i) {
        if (views[i].camera < 0) {
            cloud[i].color = UNSEEN_COLOR;
            continue;
        }
        const cv::Vec3b rgb = imagesRGB[views[i].camera].at<cv::Vec3b>(views[i].pixel);
        cloud[i].color = QVector3D(rgb[0], rgb[1], rgb[2]) / 255.0f;
        nColored++;
    }
    return nColored;
}

size_t CameraColorizer::colorizeLabels(std::vector<vaiop::QtPoint>& cloud, const std::vector<cv::Mat>& labelMaps,
                                       const std::vector<cv::Vec3b>& palette, std::vector<uint8_t>* labels) const
{
    std::vector<cv::Size> sizes(labelMaps.size());
    for (size_t c = 0; c < labelMaps.size(); ++c) {
        if (!labelMaps[c].empty() && labelMaps[c].type() == CV_8U) {
            sizes[c] = labelMaps[c].size();
        }
    }
    const std::vector<PointView> views = visibility(cloud, sizes);
    if (labels) {
        labels->assign(cloud.size(), UNSEEN_LABEL);
    }

    size_t nColored = 0;
    for (size_t i = 0; i < cloud.size(); ++i) {
        const uint8_t label = views[i].camera < 0 ? UNSEEN_LABEL : labelMaps[views[i].camera].at<uint8_t>(views[i].pixel);
        if (label >= palette.size()) {
            cloud[i].color = UNSEEN_COLOR;
            continue;
        }
        const cv::Vec3b& rgb = palette[label];
        cloud[i].color = QVector3D(rgb[0], rgb[1], rgb[2]) / 255.0f;
        if (labels) {
            (*labels)[i] = label;
        }
        nColored++;
    }
    return nColored;
}
