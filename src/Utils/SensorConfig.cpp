#include <Utils/SensorConfig.h>

#include <QCoreApplication>
#include <QDebug>
#include <QDir>
#include <QFile>
#include <QFileInfo>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>

#include <opencv2/core.hpp>

namespace {
    int rotationFromString(const QString& rotation)
    {
        const QString r = rotation.toLower();
        if (r == "cw" || r == "90") return cv::ROTATE_90_CLOCKWISE;
        if (r == "ccw" || r == "-90" || r == "270") return cv::ROTATE_90_COUNTERCLOCKWISE;
        if (r == "180") return cv::ROTATE_180;
        return -1;
    }

    QString findConfigFile(const QString& requested)
    {
        QStringList candidates;
        if (!requested.isEmpty()) {
            candidates << requested;
        }
        candidates << "config/sensors.json";
        if (QCoreApplication::instance()) {
            const QString appDir = QCoreApplication::applicationDirPath();
            candidates << appDir + "/config/sensors.json"
                       << appDir + "/../../config/sensors.json";
        }

        for (const QString& candidate : candidates) {
            if (QFileInfo::exists(candidate)) {
                return QFileInfo(candidate).absoluteFilePath();
            }
        }
        return {};
    }

    // Resolve a path in the config relative to the repository root (the parent of config/)
    std::string resolveRelative(const QString& configFile, const QString& path)
    {
        if (path.isEmpty() || QDir::isAbsolutePath(path)) {
            return path.toStdString();
        }
        QDir root = QFileInfo(configFile).absoluteDir();
        root.cdUp();
        return QDir::cleanPath(root.filePath(path)).toStdString();
    }
}

SensorConfig SensorConfig::defaults()
{
    SensorConfig config;
    config.cameras = {
        {"cam1", "192.168.2.14", cv::ROTATE_90_COUNTERCLOCKWISE, config.calibFileName},
        {"cam2", "192.168.2.13", cv::ROTATE_90_CLOCKWISE, config.calibFileName},
        {"cam3", "192.168.2.12", cv::ROTATE_90_COUNTERCLOCKWISE, config.calibFileName},
        {"cam4", "192.168.2.11", cv::ROTATE_90_CLOCKWISE, config.calibFileName},
    };
    return config;
}

SensorConfig SensorConfig::load(const std::string& path)
{
    SensorConfig config = defaults();

    const QString file = findConfigFile(QString::fromStdString(path));
    if (file.isEmpty()) {
        qWarning() << "No config/sensors.json found, using the default sensor addresses";
        return config;
    }

    QFile f(file);
    if (!f.open(QIODevice::ReadOnly)) {
        qWarning() << "Could not open" << file << "- using the default sensor addresses";
        return config;
    }
    QJsonParseError parseError;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &parseError);
    if (!doc.isObject()) {
        qWarning() << "Failed to parse" << file << ":" << parseError.errorString() << "- using the default sensor addresses";
        return config;
    }
    const QJsonObject root = doc.object();
    config.sourceFile = file.toStdString();

    if (root.contains("calibration")) {
        config.calibFileName = resolveRelative(file, root["calibration"].toString());
    } else {
        config.calibFileName = resolveRelative(file, QString::fromStdString(config.calibFileName));
    }

    config.pointCloudColor = root["pointcloud_color"].toString(QString::fromStdString(config.pointCloudColor)).toStdString();

    const QJsonObject segmentation = root["segmentation"].toObject();
    config.segmentation.modelDir = resolveRelative(file, segmentation["model"].toString());
    config.segmentation.inputSize = segmentation["input_size"].toInt(config.segmentation.inputSize);
    config.segmentation.threads = segmentation["threads"].toInt(config.segmentation.threads);
    config.segmentation.minConfidence = static_cast<float>(segmentation["min_confidence"].toDouble(config.segmentation.minConfidence));

    const QJsonObject lidar = root["lidar"].toObject();
    config.lidarIp = lidar["ip"].toString(QString::fromStdString(config.lidarIp)).toStdString();

    const QJsonObject robot = root["robot"].toObject();
    config.robotHost = robot["host"].toString(QString::fromStdString(config.robotHost)).toStdString();
    config.robotPort = robot["port"].toInt(config.robotPort);

    if (root.contains("cameras")) {
        config.cameras.clear();
        for (const QJsonValue& value : root["cameras"].toArray()) {
            const QJsonObject cam = value.toObject();
            CameraInfo info;
            info.name = cam["name"].toString(QString("cam%1").arg(config.cameras.size() + 1)).toStdString();
            info.ip = cam["ip"].toString().toStdString();
            info.rFlag = rotationFromString(cam["rotation"].toString());
            info.calibFileName = config.calibFileName;
            config.cameras.push_back(info);
        }
    } else {
        for (CameraInfo& cam : config.cameras) {
            cam.calibFileName = config.calibFileName;
        }
    }

    return config;
}

const SensorConfig& sensorConfig()
{
    static const SensorConfig config = SensorConfig::load();
    return config;
}
