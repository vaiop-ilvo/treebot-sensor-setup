#include <Gui/Recording.h>
#include <Lidar/Ouster.h>

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QHash>
#include <QTextStream>

#include <algorithm>

namespace {
    // Splits a CSV line written by RecordWidget (fields with a comma or quote are quoted, quotes doubled)
    QStringList splitCsvLine(const QString& line)
    {
        QStringList fields;
        QString field;
        bool quoted = false;
        for (int i = 0; i < line.size(); ++i) {
            const QChar c = line[i];
            if (quoted) {
                if (c == '"' && i + 1 < line.size() && line[i + 1] == '"') {
                    field += '"';
                    ++i;
                } else if (c == '"') {
                    quoted = false;
                } else {
                    field += c;
                }
            } else if (c == '"') {
                quoted = true;
            } else if (c == ',') {
                fields << field;
                field.clear();
            } else {
                field += c;
            }
        }
        fields << field;
        return fields;
    }

    // Inverse of the toIsoTime of RecordWidget; 0 when empty or invalid
    uint64_t fromIsoTime(const QString& text)
    {
        const QDateTime dateTime = QDateTime::fromString(text, "yyyy-MM-ddTHH:mm:ss.zzz");
        return dateTime.isValid() ? static_cast<uint64_t>(dateTime.toMSecsSinceEpoch()) * 1000 : 0;
    }
}

bool Recording::open(const QString& folder, QString& error)
{
    QDir dir(folder);
    // Several logs only appear when a folder was reused by hand; take the most recent one
    const QStringList logs = dir.entryList({"sensor_log_*.csv"}, QDir::Files, QDir::Name);
    if (logs.isEmpty()) {
        error = QString("No sensor_log_*.csv in\n%1").arg(folder);
        return false;
    }

    QFile file(dir.filePath(logs.last()));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        error = QString("Cannot read %1\n%2").arg(file.fileName(), file.errorString());
        return false;
    }
    QTextStream stream(&file);

    // Columns by name, so logs with another number of cameras or extra columns still work
    QHash<QString, int> columns;
    const QStringList header = splitCsvLine(stream.readLine());
    for (int i = 0; i < header.size(); ++i) {
        columns.insert(header[i].trimmed(), i);
    }
    int cameraCount = 0;
    while (columns.contains(QString("cam%1_ip").arg(cameraCount + 1))) {
        cameraCount++;
    }
    if (!columns.contains("lidar_pc_path") && cameraCount == 0) {
        error = QString("%1 is not a sensor log (no lidar or camera columns)").arg(file.fileName());
        return false;
    }

    std::vector<RecordedFrame> newRows;
    std::vector<uint64_t> newTimes;
    while (!stream.atEnd()) {
        const QString line = stream.readLine();
        if (line.trimmed().isEmpty()) {
            continue;
        }
        const QStringList fields = splitCsvLine(line);
        auto field = [&](const QString& name) {
            const int index = columns.value(name, -1);
            return index >= 0 && index < fields.size() ? fields[index] : QString();
        };

        RecordedFrame frame;
        frame.row = newRows.size();

        frame.lidar.ip = field("lidar_ip").toStdString();
        frame.lidar.ts = fromIsoTime(field("lidar_ts"));
        frame.lidar.valid = frame.lidar.ts != 0;
        frame.lidar.frameId = field("lidar_frame_id").toULongLong();
        frame.lidar.fps = field("lidar_fps").toDouble();
        frame.lidar.pcPath = field("lidar_pc_path").toStdString();

        frame.cameras.resize(cameraCount);
        for (int i = 0; i < cameraCount; ++i) {
            const QString cam = QString("cam%1").arg(i + 1);
            CameraMetaData& camMetaData = frame.cameras[i];
            camMetaData.ip = field(cam + "_ip").toStdString();
            camMetaData.ts = fromIsoTime(field(cam + "_ts"));
            camMetaData.valid = camMetaData.ts != 0;
            camMetaData.frameId = field(cam + "_frame_id").toULongLong();
            camMetaData.fps = field(cam + "_fps").toDouble();
            camMetaData.imPath = field(cam + "_im_path").toStdString();
        }

        RobotMetaData& robot = frame.robot;
        robot.valid = !field("robot_x").isEmpty();
        robot.x = field("robot_x").toDouble();
        robot.y = field("robot_y").toDouble();
        robot.z = field("robot_z").toDouble();
        robot.roll = field("robot_roll").toDouble();
        robot.pitch = field("robot_pitch").toDouble();
        robot.yaw = field("robot_yaw").toDouble();
        robot.lat = field("robot_lat").toDouble();
        robot.lon = field("robot_lon").toDouble();
        robot.gpsFix = field("robot_gps_fix").toInt();
        robot.gpsLngSpeed = field("robot_gps_lng_speed").toDouble();
        robot.odomLngSpeed = field("robot_odom_lng_speed").toDouble();
        robot.odomAngSpeed = field("robot_odom_ang_speed").toDouble();

        // Play back on the trigger (lidar) time, the log time when the row has no lidar scan
        uint64_t ts = frame.lidar.ts != 0 ? frame.lidar.ts : fromIsoTime(field("timestamp"));
        if (!newTimes.empty()) {
            ts = std::max(ts, newTimes.back());
        }
        newTimes.push_back(ts);
        newRows.push_back(std::move(frame));
    }

    if (newRows.empty()) {
        error = QString("%1 has no rows").arg(file.fileName());
        return false;
    }

    // Relative to the first row
    const uint64_t start = newTimes.front();
    for (uint64_t& ts : newTimes) {
        ts -= start;
    }

    recordFolder = dir.absolutePath();
    logFileName = file.fileName();
    nCameras = cameraCount;
    rows = std::move(newRows);
    times = std::move(newTimes);
    return true;
}

QString Recording::folder() const
{
    return recordFolder;
}

QString Recording::logFile() const
{
    return logFileName;
}

size_t Recording::size() const
{
    return rows.size();
}

int Recording::cameraCount() const
{
    return nCameras;
}

uint64_t Recording::time(size_t row) const
{
    return row < times.size() ? times[row] : (times.empty() ? 0 : times.back());
}

size_t Recording::rowAt(uint64_t time) const
{
    if (times.empty()) {
        return 0;
    }
    const auto it = std::upper_bound(times.begin(), times.end(), time);
    return it == times.begin() ? 0 : static_cast<size_t>(it - times.begin()) - 1;
}

RecordedFrame Recording::load(size_t row) const
{
    RecordedFrame frame = rows.at(row);
    const QDir dir(recordFolder);

    if (!frame.lidar.pcPath.empty()) {
        frame.pointCloud = Ouster::loadPLY(dir.filePath(QString::fromStdString(frame.lidar.pcPath)).toStdString());
    }

    frame.images.resize(frame.cameras.size());
    for (size_t i = 0; i < frame.cameras.size(); ++i) {
        if (!frame.cameras[i].imPath.empty()) {
            frame.images[i].load(dir.filePath(QString::fromStdString(frame.cameras[i].imPath)));
        }
    }
    return frame;
}
