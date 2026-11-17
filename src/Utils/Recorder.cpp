#include <Utils/Recorder.h>

#include <QDateTime>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QString>

#include <algorithm>
#include <chrono>

namespace {
    uint64_t hostTimeMicros()
    {
        return std::chrono::time_point_cast<std::chrono::microseconds>(std::chrono::system_clock::now()).time_since_epoch().count();
    }

    std::string toJson(const QJsonObject& object)
    {
        return QJsonDocument(object).toJson(QJsonDocument::Compact).toStdString();
    }

    // /record/row: everything of one CSV row, so scripts/bag_to_folder.py can write the folder format from the bag.
    // cloudStamp / imageStamps: stamps of the messages in the bag, 0 when not written.
    std::string rowJson(uint64_t row, const QString& logTime, const Frame& frame, uint64_t cloudStamp,
                        const std::vector<uint64_t>& imageStamps)
    {
        auto stamp = [](uint64_t us) { return us ? QJsonValue(static_cast<double>(us)) : QJsonValue(); };
        const OusterMetaData& lidar = frame.lidar;
        const QJsonObject lidarJson{{"valid", lidar.valid}, {"ip", QString::fromStdString(lidar.ip)}, {"ts", static_cast<double>(lidar.ts)},
                                    {"frame_id", static_cast<double>(lidar.frameId)}, {"fps", lidar.fps}, {"cloud_stamp", stamp(cloudStamp)}};
        QJsonArray camerasJson;
        for (size_t i = 0; i < frame.cameras.size(); ++i) {
            const CameraMetaData& cam = frame.cameras[i].meta;
            camerasJson.append(QJsonObject{{"valid", cam.valid}, {"ip", QString::fromStdString(frame.cameras[i].ip)},
                                           {"ts", static_cast<double>(cam.ts)}, {"frame_id", static_cast<double>(cam.frameId)},
                                           {"fps", cam.fps}, {"image_stamp", stamp(i < imageStamps.size() ? imageStamps[i] : 0)}});
        }
        const RobotMetaData& robot = frame.robot;
        QJsonObject redis;
        for (const auto& [key, raw] : robot.redis) {
            redis.insert(QString::fromStdString(key), raw ? QJsonValue(QString::fromStdString(*raw)) : QJsonValue());
        }
        const QJsonObject robotJson{{"valid", robot.valid}, {"x", robot.x}, {"y", robot.y}, {"z", robot.z}, {"roll", robot.roll},
                                    {"pitch", robot.pitch}, {"yaw", robot.yaw}, {"lat", robot.lat}, {"lon", robot.lon},
                                    {"gps_fix", robot.gpsFix}, {"gps_lng_speed", robot.gpsLngSpeed},
                                    {"odom_lng_speed", robot.odomLngSpeed}, {"odom_ang_speed", robot.odomAngSpeed}, {"redis", redis}};
        return toJson(QJsonObject{{"row", static_cast<double>(row)}, {"log_time", logTime},
                                  {"trigger_ts", static_cast<double>(frame.triggerTs)}, {"lidar", lidarJson},
                                  {"cameras", camerasJson}, {"robot", robotJson}});
    }
}

std::vector<std::string> extraRobotKeys(const std::vector<RedisKey>& keys)
{
    const std::vector<RedisKey> known = SensorConfig::defaultRobotKeys();
    std::vector<std::string> extra;
    for (const RedisKey& key : keys) {
        if (std::none_of(known.begin(), known.end(), [&key](const RedisKey& k) { return k.key == key.key; })) {
            extra.push_back(key.key);
        }
    }
    return extra;
}

std::vector<std::string> folderLogHeader(int nCameras, const std::vector<std::string>& extraRobotKeys)
{
    std::vector<std::string> header = {"timestamp", "lidar_ip", "lidar_ts", "lidar_frame_id", "lidar_fps", "lidar_pc_path"};
    for (int i = 1; i <= nCameras; ++i) {
        const std::string cam = "cam" + std::to_string(i);
        for (const char* field : {"_ip", "_ts", "_frame_id", "_fps", "_im_path"}) {
            header.push_back(cam + field);
        }
    }
    for (const char* field : {"robot_x", "robot_y", "robot_z", "robot_roll", "robot_pitch", "robot_yaw",
                              "robot_lat", "robot_lon", "robot_gps_fix", "robot_gps_lng_speed",
                              "robot_odom_lng_speed", "robot_odom_ang_speed"}) {
        header.push_back(field);
    }
    for (const std::string& key : extraRobotKeys) {
        header.push_back("redis:" + key);
    }
    return header;
}

Recorder::Recorder(const SensorConfig& config, std::vector<CameraCalib> calibs)
    : config(config),
      calibs(std::move(calibs))
{
}

void Recorder::setClassNames(std::vector<std::string> names)
{
    std::lock_guard<std::mutex> lock(mutex);
    classNames = std::move(names);
}

bool Recorder::start(const std::string& folder, std::string& error)
{
    std::lock_guard<std::mutex> lock(mutex);
    const QString uri = QString::fromStdString(folder) + "/rosbag_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss");
    if (!bag.open(uri.toStdString(), error)) {
        return false;
    }
    rowCount = 0;

    // CSV name and header for scripts/bag_to_folder.py, and the camera names (= topic names)
    const uint64_t now = hostTimeMicros();
    QJsonArray header;
    for (const std::string& column : folderLogHeader(static_cast<int>(config.cameras.size()), extraRobotKeys(config.robotKeys))) {
        header.append(QString::fromStdString(column));
    }
    QJsonArray cameraNames;
    for (const CameraInfo& camera : config.cameras) {
        cameraNames.append(QString::fromStdString(camera.name));
    }
    const QString csvName = "sensor_log_" + QDateTime::currentDateTime().toString("yyyyMMdd_hhmmss") + ".csv";
    bag.writeRecordInfo(now, toJson(QJsonObject{{"format_version", 1}, {"csv_name", csvName}, {"csv_header", header},
                                                {"cameras", cameraNames}}));
    bag.writeCalibration(now, calibs);
    if (!classNames.empty()) {
        bag.writeClassNames(now, classNames);
    }
    recording = true;
    return true;
}

void Recorder::stop()
{
    std::lock_guard<std::mutex> lock(mutex);
    recording = false;
    bag.close();
}

bool Recorder::isRecording() const
{
    return recording;
}

std::string Recorder::uri() const
{
    return bag.uri();
}

uint64_t Recorder::rows() const
{
    return rowCount;
}

size_t Recorder::pending() const
{
    return bag.pending();
}

size_t Recorder::dropped() const
{
    return bag.dropped();
}

void Recorder::writeFrame(const Frame& frame, const GroundPlane& plane, std::shared_ptr<const OccupancyMap> occupancy, bool withGridMap)
{
    if (!recording || !bag.isOpen()) {
        return;     // Stopped while the frame was on its way
    }
    const uint64_t row = rowCount++;
    const uint64_t stamp = frame.triggerTs;

    uint64_t cloudStamp = 0;
    if (frame.cloud) {
        bag.writeScan(stamp, frame.cloud);
        cloudStamp = stamp;
        bag.writeGroundPlane(stamp, plane);
        bag.writeOccupancy(stamp, occupancy, true, withGridMap);
    }
    if (frame.robot.valid) {
        bag.writeRobot(stamp, frame.robot);
    }
    std::vector<uint64_t> imageStamps(frame.cameras.size(), 0);
    for (size_t i = 0; i < frame.cameras.size(); ++i) {
        const CameraSample& camera = frame.cameras[i];
        if (camera.newFrame) {
            bag.writeImage(i, camera.meta.ts, camera.image);
            imageStamps[i] = camera.meta.ts;
        }
    }
    // The row of the CSV log; "timestamp" = when the row is complete
    const QString logTime = QDateTime::currentDateTime().toString("yyyy-MM-ddTHH:mm:ss.zzz");
    bag.writeRow(stamp, rowJson(row, logTime, frame, cloudStamp, imageStamps));
}

void Recorder::writeSegmentation(const Frame& frame, std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud,
                                 const std::vector<uint8_t>& labels, std::shared_ptr<const OccupancyMap> classMap)
{
    if (!frame.record || !recording || !bag.isOpen()) {
        return;
    }
    bag.writeSegmentedScan(frame.triggerTs, cloud, labels);
    bag.writeOccupancy(frame.triggerTs, classMap, false, true);
}
