#include <Lidar/Ouster.h>
#include <Utils/AsyncWriter.h>
#include <Utils/PointCloudColor.h>
#include <QDebug>
#include <QString>
#include <algorithm>
#include <cmath>
#include <iostream>

Ouster::Ouster(const std::string& ip) : ip(ip)
{
    // Limit ouster_client log statements to "info"
    ouster::sdk::sensor::init_logger("info");

    qDebug() << QString("Connecting to %1").arg(QString::fromStdString(ip));

    // Finally create the client. This will configure all the sensors
    // and wait for them to initialize.
    // After this, the source immediately begins collecting data in the
    // background
    sourceClient = std::make_unique<ouster::sdk::sensor::SensorScanSource>(ip);
    httpClient = std::make_unique<CurlClient>(ip);

    // Now we can print metadata about each sensor since it was collected by the
    // source already While we are at it build necessary lookup tables
    for (const auto& info: sourceClient->sensor_info()) {
        info_ = *info;

        size_t w = info_.format.columns_per_frame;
        size_t h = info_.format.pixels_per_column;

        ouster::sdk::core::ColumnWindow column_window = info_.format.column_window;

        qDebug().noquote() << QString(
            "Sensor %1 \n"
            " Firmware version: %2\n"
            " Serial number: %3\n"
            " Product line: %4\n"
            " Scan dimensions: %5 x %6\n"
            " Column window: [%7, %8]")
            .arg(QString::fromStdString(ip))
            .arg(QString::fromStdString(info_.image_rev))
            .arg(info_.sn)
            .arg(QString::fromStdString(info_.prod_line))
            .arg(QString::number(w))
            .arg(QString::number(h))
            .arg(QString::number(column_window.first))
            .arg(QString::number(column_window.second));

        // Pre-compute a table for efficiently calculating point clouds from
        // range
        xyz_lut_ = ouster::sdk::core::make_xyz_lut(info_, true /* if extrinsics should be used or not */);
    }

    // Timing for fps
    tic = std::chrono::steady_clock::now();
}

void Ouster::resetTimestamp()
{
    // Wait for a scan so ts holds a real sensor time.
    // A scan performs better than the http readout of the system time (api/v1/time/system).
    for (int attempt = 0; attempt < 10 && !scan(0.2); ++attempt) {}

    auto now = std::chrono::system_clock::now();
    tsRef = std::chrono::time_point_cast<std::chrono::microseconds>(now).time_since_epoch().count() - ts;
}

bool Ouster::scan(double timeoutSec)
{
    auto result = sourceClient->get_scan(timeoutSec);

    // Do not overwrite the last scan when the queue is empty.
    if (!result.second) {
        return false;
    }

    scan_ = std::move(result.second);

    // Update frameId
    uint64_t newFrameId = scan_->frame_id;
    if (frameId == newFrameId && scanCount > 0) {
        return false;  // No new scan available
    }

    // Update fps
    auto toc = std::chrono::steady_clock::now();
    std::chrono::duration<double> duration = toc - tic;
    if (scanCount > 0 && duration.count() > 0.0) {
        fps = 1.0 / duration.count();
    }
    tic = toc;
    frameId = newFrameId;
    scanCount++;

    // Update timestamp from the first valid column
    auto status = scan_->status();
    auto it = std::find_if(status.data(), status.data() + status.size(),
                            [](const uint32_t s) {
                                return (s & 0x01);
                            });
    if (it != status.data() + status.size()) {
        ts = std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::nanoseconds(scan_->timestamp()(
                it - status.data()))).count();
    }

    return true;  // Successfully scanned
}


std::shared_ptr<std::vector<vaiop::QtPoint>> Ouster::getPointCloud()
{
    if (!scan_) return nullptr;

    // First compute a point cloud using the lookup table
    auto ousterCloud = ouster::sdk::core::cartesian(*scan_, xyz_lut_);
    auto qCloud = std::make_shared<std::vector<vaiop::QtPoint>>();
    auto newPclCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();

    const Eigen::Index nPoints = ousterCloud.rows();
    qCloud->reserve(nPoints);
    newPclCloud->reserve(nPoints);
    eigenCloud.resize(nPoints, 4);

    // write each point, filtering out points without returns
    Eigen::Index nValid = 0;
    for (Eigen::Index i = 0; i < nPoints; i++) {
        auto xyz = ousterCloud.row(i);
        if (xyz.isApproxToConstant(0.0)) {
            continue;
        }

        const float x = static_cast<float>(xyz.x());
        const float y = static_cast<float>(xyz.y());
        const float z = static_cast<float>(xyz.z());

        // Populate Qt cloud
        vaiop::QtPoint p;
        p.position = QVector3D(x, y, z);
        qCloud->push_back(p);

        // Populate PCL cloud
        newPclCloud->push_back(pcl::PointXYZ(x, y, z));

        // Populate Eigen matrix
        eigenCloud.row(nValid) << xyz.x(), xyz.y(), xyz.z(), 1.0;
        nValid++;
    }
    eigenCloud.conservativeResize(nValid, 4);
    pclCloud = newPclCloud;
    colorByDistance(*qCloud);

    return qCloud;
}

std::shared_ptr<cv::Mat> Ouster::getReflectivityImage()
{
    if (!scan_) return nullptr;

    // Get the raw (staggered) reflectivity data
    const auto& reflectivityStaggered = scan_->field<uint8_t>(ouster::sdk::core::ChanField::REFLECTIVITY);

    // Destagger
    auto reflectivityDestaggered = ouster::sdk::core::destagger<uint8_t>(reflectivityStaggered, info_.format.pixel_shift_by_row);

    // Wrap as 8-bit single-channel image and deep copy: the Eigen buffer is freed when this function returns
    cv::Mat view(
        static_cast<int>(reflectivityDestaggered.rows()),
        static_cast<int>(reflectivityDestaggered.cols()),
        CV_8UC1,
        reflectivityDestaggered.data()
    );

    return std::make_shared<cv::Mat>(view.clone());
}

const Eigen::MatrixXd& Ouster::getEigenCloud() const
{
    return eigenCloud;
}

const std::string& Ouster::getIp() const
{
    return ip;
}

double Ouster::getFps() const
{
    return fps;
}

uint64_t Ouster::getFrameId() const
{
    return frameId;
}

uint64_t Ouster::getScanCount() const
{
    return scanCount;
}

uint64_t Ouster::getTimeStamp() const
{
    return tsRef + ts;
}

void Ouster::getMetaData(OusterMetaData& metaData) const
{
    metaData.valid = scan_ != nullptr;
    metaData.frameId = getFrameId();
    metaData.ts = getTimeStamp();
    metaData.fps = fps;
    metaData.ip = ip;
}


bool Ouster::savePLY(const std::string& fileName) const
{
    if (!pclCloud) {
        std::cout << "No point cloud available to save (call getPointCloud first)." << std::endl;
        return false;
    }

    // The cloud is replaced (not modified) by getPointCloud, so sharing it with the writer is safe
    pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud = pclCloud;
    return AsyncWriter::instance().enqueue([fileName, cloud]() {
        if (pcl::io::savePLYFileBinary(fileName, *cloud) != 0) {
            std::cerr << "Failed to write " << fileName << std::endl;
        }
    });
}

std::shared_ptr<std::vector<vaiop::QtPoint>> Ouster::loadPLY(const std::string& fileName)
{
    pcl::PointCloud<pcl::PointXYZ> cloud;
    if (pcl::io::loadPLYFile(fileName, cloud) != 0) {
        return nullptr;
    }

    auto qCloud = std::make_shared<std::vector<vaiop::QtPoint>>();
    qCloud->reserve(cloud.size());
    for (const pcl::PointXYZ& point : cloud) {
        vaiop::QtPoint p;
        p.position = QVector3D(point.x, point.y, point.z);
        qCloud->push_back(p);
    }
    colorByDistance(*qCloud);
    return qCloud;
}
