#include <Utils/LidarCamera.h>

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <opencv2/core/eigen.hpp>

namespace {
    bool readMatrix(const QJsonArray& rows, Eigen::MatrixXd& matrix)
    {
        if (rows.size() != matrix.rows()) {
            return false;
        }
        for (int i = 0; i < matrix.rows(); ++i) {
            const QJsonArray row = rows[i].toArray();
            if (row.size() != matrix.cols()) {
                return false;
            }
            for (int j = 0; j < matrix.cols(); ++j) {
                matrix(i, j) = row[j].toDouble();
            }
        }
        return true;
    }

    // Largest radius on the normalised image plane up to which the radial distortion keeps increasing.
    // Beyond it the polynomial folds back, so points far outside the field of view would land in the image.
    double monotonicRadius(const cv::Mat& dist)
    {
        auto k = [&dist](int i) { return i < static_cast<int>(dist.total()) ? dist.at<double>(i) : 0.0; };
        auto distorted = [&](double r) {
            const double r2 = r * r;
            const double num = 1.0 + r2 * (k(0) + r2 * (k(1) + r2 * k(4)));
            const double den = 1.0 + r2 * (k(5) + r2 * (k(6) + r2 * k(7)));
            return r * num / den;
        };
        constexpr double STEP = 0.002;
        constexpr double MAX_RADIUS = 10.0;
        double previous = 0.0;
        for (double r = STEP; r < MAX_RADIUS; r += STEP) {
            const double rd = distorted(r);
            if (rd <= previous) {
                return r - STEP;
            }
            previous = rd;
        }
        return MAX_RADIUS;
    }

    // Undistorted extent of the image border on the normalised image plane
    void computeImageBounds(CameraCalib& calib)
    {
        const double rMax = monotonicRadius(calib.distCoeffs);
        calib.maxRadius2 = rMax * rMax;
        if (calib.imageSize.area() == 0) {
            return;
        }
        const int w = calib.imageSize.width;
        const int h = calib.imageSize.height;
        std::vector<cv::Point2d> border;
        for (int u = 0; u < w; u += 8) {
            border.emplace_back(u, 0);
            border.emplace_back(u, h - 1);
        }
        for (int v = 0; v < h; v += 8) {
            border.emplace_back(0, v);
            border.emplace_back(w - 1, v);
        }
        cv::Mat K;
        cv::eigen2cv(calib.K, K);
        // The default undistortPoints stops after 5 iterations, which does not converge in the corners of a wide lens
        std::vector<cv::Point2d> normalised;
        cv::undistortPoints(border, normalised, K, calib.distCoeffs, cv::noArray(), cv::noArray(),
                            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-10));

        calib.xMin = calib.yMin = 1e9;
        calib.xMax = calib.yMax = -1e9;
        for (const cv::Point2d& p : normalised) {
            calib.xMin = std::min(calib.xMin, std::max(p.x, -rMax));
            calib.xMax = std::max(calib.xMax, std::min(p.x, rMax));
            calib.yMin = std::min(calib.yMin, std::max(p.y, -rMax));
            calib.yMax = std::max(calib.yMax, std::min(p.y, rMax));
        }
        // Small margin: the exact border is checked again after projection
        const double mx = 0.02 * (calib.xMax - calib.xMin);
        const double my = 0.02 * (calib.yMax - calib.yMin);
        calib.xMin -= mx; calib.xMax += mx;
        calib.yMin -= my; calib.yMax += my;
    }
}

CameraCalib loadCameraCalib(const std::string& calibFileName, const std::string& camName)
{
    CameraCalib calib;
    calib.name = camName;
    if (calibFileName.empty()) {
        return calib;
    }

    QFile file(QString::fromStdString(calibFileName));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Could not open calibration file" << file.fileName();
        return calib;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (doc.isNull() || !doc.isObject()) {
        qWarning() << "Failed to parse calibration file" << file.fileName();
        return calib;
    }

    const QJsonObject rootObj = doc.object();
    const QString cam = QString::fromStdString(camName);

    // Intrinsics and lidar to camera extrinsics are required
    Eigen::MatrixXd K(3, 3), T(4, 4);
    if (!readMatrix(rootObj["intrinsics"].toObject()[cam].toArray(), K)
        || !readMatrix(rootObj["extrinsics"].toObject()[cam].toArray(), T)) {
        qWarning() << "No valid calibration for" << cam << "in" << file.fileName();
        return calib;
    }
    calib.K = K;
    calib.tLidarCam = T;

    // Optional: distortion, image size and rotation (older files only hold K and T)
    const QJsonArray dist = rootObj["distortion"].toObject()[cam].toArray();
    if (!dist.isEmpty()) {
        calib.distCoeffs = cv::Mat(1, static_cast<int>(dist.size()), CV_64F);
        for (int i = 0; i < dist.size(); ++i) {
            calib.distCoeffs.at<double>(0, i) = dist[i].toDouble();
        }
    }
    const QJsonArray size = rootObj["image_size"].toObject()[cam].toArray();
    if (size.size() == 2) {
        calib.imageSize = cv::Size(size[0].toInt(), size[1].toInt());
    }
    const QJsonValue rotation = rootObj["rotation_deg"].toObject()[cam];
    if (rotation.isDouble()) {
        calib.rotationDeg = rotation.toInt();
    }

    computeImageBounds(calib);
    calib.valid = true;
    return calib;
}

bool loadCalib(const std::string& calibFileName, const std::string& camName, Eigen::Matrix3d& K, Eigen::Matrix4d& tLidarCam)
{
    const CameraCalib calib = loadCameraCalib(calibFileName, camName);
    K = calib.K;
    tLidarCam = calib.tLidarCam;
    return calib.valid;
}
