#include <Utils/LidarCamera.h>

#include <QDebug>
#include <QFile>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

std::shared_ptr<std::vector<vaiop::QtPoint>> projectLidarCamera(const Eigen::MatrixXd& pLidar, const cv::Mat& im, const Eigen::Matrix4d& tLidarCam, const Eigen::Matrix3d& K)
{
    auto qtCloud = std::make_shared<std::vector<vaiop::QtPoint>>();
    qtCloud->reserve(pLidar.rows());

    // Projection matrix for column vectors: p_pixel = Kh * T * p_lidar
    Eigen::Matrix4d Kh = Eigen::Matrix4d::Identity();
    Kh.block<3,3>(0,0) = K;
    Eigen::Matrix4d Pt = Kh * tLidarCam;
    // Points are stored as rows, so transpose: P_pixel = P_lidar * Pt^T
    Eigen::MatrixXd pPixel = pLidar * Pt.transpose();

    for (Eigen::Index i = 0; i < pPixel.rows(); ++i) {
        vaiop::QtPoint p;
        p.position = QVector3D(pLidar(i, 0), pLidar(i, 1), pLidar(i, 2));
        p.color = QVector3D(0.5f, 0.5f, 0.5f);

        // Only points in front of the camera can be projected
        const double depth = pPixel(i, 2);
        if (depth > 0.0) {
            // Normalize with the scaling factor to get pixel (u,v)
            const double u = pPixel(i, 0) / depth;
            const double v = pPixel(i, 1) / depth;

            if (u >= 0 && u < im.cols && v >= 0 && v < im.rows) {
                cv::Vec3b color = im.at<cv::Vec3b>(static_cast<int>(v), static_cast<int>(u));
                // BGR -> RGB in [0, 1]
                p.color = QVector3D(color[2], color[1], color[0]) / 255.0f;
            }
        }

        qtCloud->push_back(p);
    }

    return qtCloud;
}


bool loadCalib(const std::string& calibFileName, const std::string& camName, Eigen::Matrix3d& K, Eigen::Matrix4d& tLidarCam)
{
    // Default
    K = Eigen::Matrix3d::Identity();
    tLidarCam = Eigen::Matrix4d::Identity();

    if (calibFileName.empty()) {
        return false;
    }

    QFile file(QString::fromStdString(calibFileName));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qWarning() << "Could not open calibration file" << file.fileName();
        return false;
    }

    QJsonDocument doc = QJsonDocument::fromJson(file.readAll());
    if (doc.isNull() || !doc.isObject()) {
        qWarning() << "Failed to parse calibration file" << file.fileName();
        return false;
    }

    const QJsonObject rootObj = doc.object();
    const QString cam = QString::fromStdString(camName);
    const QJsonArray intrinsics = rootObj["intrinsics"].toObject()[cam].toArray();
    const QJsonArray extrinsics = rootObj["extrinsics"].toObject()[cam].toArray();
    if (intrinsics.size() != 3 || extrinsics.size() != 4) {
        qWarning() << "No valid calibration for" << cam << "in" << file.fileName();
        return false;
    }

    // Parse intrinsics
    for (int i = 0; i < 3; ++i) {
        QJsonArray row = intrinsics[i].toArray();
        for (int j = 0; j < 3; ++j) {
            K(i, j) = row[j].toDouble();
        }
    }

    // Parse lidar to camera extrinsics
    for (int i = 0; i < 4; ++i) {
        QJsonArray row = extrinsics[i].toArray();
        for (int j = 0; j < 4; ++j) {
            tLidarCam(i, j) = row[j].toDouble();
        }
    }
    return true;
}
