#pragma once

#include <QImage>
#include <QString>

#include <memory>
#include <vector>

#include <Lidar/QtPoint.h>
#include <Utils/MetaData.hpp>

// The data of one logged row, read back from disk
struct RecordedFrame {
    size_t row = 0;
    OusterMetaData lidar;
    std::shared_ptr<std::vector<vaiop::QtPoint>> pointCloud;   // nullptr when the row has no (readable) point cloud
    std::vector<CameraMetaData> cameras;
    std::vector<QImage> images;                                 // Null image when the row has no (readable) image
    RobotMetaData robot;
};

// A recording folder written by RecordWidget: the sensor_log_*.csv plus the lidar/ and camN/ files it refers to
class Recording {
public:
    // Reads the log of the folder. Returns false and sets error when the folder holds no readable recording.
    bool open(const QString& folder, QString& error);

    QString folder() const;
    QString logFile() const;
    size_t size() const;
    int cameraCount() const;

    // Time of the row in microseconds since the first row; never decreases from one row to the next
    uint64_t time(size_t row) const;
    // Last row at or before the given time (microseconds since the first row)
    size_t rowAt(uint64_t time) const;

    // Reads the files of the row. Safe to call from another thread.
    RecordedFrame load(size_t row) const;

private:
    QString recordFolder;
    QString logFileName;
    int nCameras = 0;
    std::vector<RecordedFrame> rows;        // Metadata only, the files are read by load()
    std::vector<uint64_t> times;
};
