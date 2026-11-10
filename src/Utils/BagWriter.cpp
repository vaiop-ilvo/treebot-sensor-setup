#include <Utils/BagWriter.h>

#include <Utils/OccupancyMap.h>

#ifdef WITH_ROSBAG
#include <condition_variable>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <filesystem>
#include <functional>
#include <iostream>
#include <limits>
#include <mutex>
#include <thread>

#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/time.hpp>
#include <rosbag2_cpp/writer.hpp>
#include <rosbag2_storage/storage_options.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
#ifdef WITH_GRID_MAP
#include <grid_map_msgs/msg/grid_map.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#endif

namespace {
    constexpr size_t THREADS = 3;           // PNG compression of 4 cameras at 10 Hz needs more than one
    constexpr size_t MAX_QUEUED = 400;      // Tasks; ~10 s of data
    const char* LIDAR_FRAME = "lidar";
    const char* GROUND_FRAME = "ground";

    builtin_interfaces::msg::Time toStamp(uint64_t us)
    {
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(us / 1000000);
        t.nanosec = static_cast<uint32_t>((us % 1000000) * 1000);
        return t;
    }

    // Frame and topic name of a camera: its name in the calibration (cam1, ...), else from its index
    std::string cameraName(size_t camera, const std::vector<CameraCalib>& calibs = {})
    {
        if (camera < calibs.size() && !calibs[camera].name.empty()) {
            return calibs[camera].name;
        }
        return "cam" + std::to_string(camera + 1);
    }

    sensor_msgs::msg::PointField field(const std::string& name, uint32_t offset, uint8_t datatype)
    {
        sensor_msgs::msg::PointField f;
        f.name = name;
        f.offset = offset;
        f.datatype = datatype;
        f.count = 1;
        return f;
    }

    geometry_msgs::msg::Quaternion toQuaternion(const Eigen::Quaterniond& q)
    {
        geometry_msgs::msg::Quaternion m;
        m.x = q.x();
        m.y = q.y();
        m.z = q.z();
        m.w = q.w();
        return m;
    }

    // ground -> lidar: the rows of the rotation are the ground axes (forward, left, up) in the lidar frame
    geometry_msgs::msg::TransformStamped groundTransform(uint64_t stampUs, const GroundPlane& plane)
    {
        Eigen::Vector3d up = plane.normal.cast<double>().normalized();
        Eigen::Vector3d forward = Eigen::Vector3d::UnitX() - up.x() * up;
        if (forward.norm() < 1e-3) {
            forward = Eigen::Vector3d::UnitY() - up.y() * up;
        }
        forward.normalize();
        const Eigen::Vector3d left = up.cross(forward);
        Eigen::Matrix3d groundFromLidar;
        groundFromLidar << forward.transpose(), left.transpose(), up.transpose();

        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = toStamp(stampUs);
        t.header.frame_id = GROUND_FRAME;
        t.child_frame_id = LIDAR_FRAME;
        t.transform.translation.z = plane.sensorHeight();
        t.transform.rotation = toQuaternion(Eigen::Quaterniond(groundFromLidar));
        return t;
    }

    nav_msgs::msg::OccupancyGrid toOccupancyGrid(uint64_t stampUs, const OccupancyMap& map)
    {
        nav_msgs::msg::OccupancyGrid grid;
        grid.header.stamp = toStamp(stampUs);
        grid.header.frame_id = GROUND_FRAME;
        grid.info.map_load_time = grid.header.stamp;
        grid.info.resolution = map.cellSize;
        grid.info.width = static_cast<uint32_t>(map.cellsX);
        grid.info.height = static_cast<uint32_t>(map.cellsY);
        grid.info.origin.position.x = map.originX;
        grid.info.origin.position.y = map.originY;
        grid.info.origin.orientation.w = 1.0;
        grid.data = map.occupancy;
        return grid;
    }

#ifdef WITH_GRID_MAP
    // grid_map layout: matrix rows along x, columns along y, index (0, 0) at the corner with the largest x and y,
    // stored column major (as grid_map_ros::GridMapRosConverter::toMessage)
    grid_map_msgs::msg::GridMap toGridMap(uint64_t stampUs, const OccupancyMap& map)
    {
        grid_map_msgs::msg::GridMap msg;
        msg.header.stamp = toStamp(stampUs);
        msg.header.frame_id = GROUND_FRAME;
        msg.info.resolution = map.cellSize;
        msg.info.length_x = map.cellsX * map.cellSize;
        msg.info.length_y = map.cellsY * map.cellSize;
        msg.info.pose.position.x = map.originX + 0.5 * msg.info.length_x;
        msg.info.pose.position.y = map.originY + 0.5 * msg.info.length_y;
        msg.info.pose.orientation.w = 1.0;
        msg.layers = {"occupancy", "elevation"};
        if (map.hasClasses) {
            msg.layers.push_back("class");
        }
        msg.basic_layers = {"occupancy"};

        const size_t rows = static_cast<size_t>(map.cellsX);
        const size_t cols = static_cast<size_t>(map.cellsY);
        auto layer = [&](auto valueOf) {
            std_msgs::msg::Float32MultiArray array;
            array.layout.dim.resize(2);
            array.layout.dim[0].label = "column_index";
            array.layout.dim[0].size = static_cast<uint32_t>(cols);
            array.layout.dim[0].stride = static_cast<uint32_t>(rows * cols);
            array.layout.dim[1].label = "row_index";
            array.layout.dim[1].size = static_cast<uint32_t>(rows);
            array.layout.dim[1].stride = static_cast<uint32_t>(rows);
            array.data.resize(rows * cols);
            for (size_t col = 0; col < cols; ++col) {
                const size_t iy = cols - 1 - col;
                for (size_t row = 0; row < rows; ++row) {
                    const size_t ix = rows - 1 - row;
                    array.data[col * rows + row] = valueOf(iy * rows + ix);
                }
            }
            return array;
        };
        const float nan = std::numeric_limits<float>::quiet_NaN();
        msg.data.push_back(layer([&](size_t cell) {
            return map.occupancy[cell] == OccupancyMap::UNKNOWN ? nan : static_cast<float>(map.occupancy[cell]);
        }));
        msg.data.push_back(layer([&](size_t cell) { return map.elevation[cell]; }));
        if (map.hasClasses) {
            msg.data.push_back(layer([&](size_t cell) {
                return map.classes[cell] == BirdEyePoint::NO_LABEL ? nan : static_cast<float>(map.classes[cell]);
            }));
        }
        return msg;
    }
#endif
}

struct BagWriter::Impl {
    mutable std::mutex mutex;           // Queue and state
    std::condition_variable wakeUp;
    std::condition_variable idle;
    std::deque<std::function<void()>> tasks;
    std::vector<std::thread> threads;
    size_t active = 0;
    size_t droppedCount = 0;
    bool stopping = false;
    bool open = false;
    std::string bagUri;
    std::vector<CameraCalib> calibs;    // For the camera_info of the images

    std::mutex writeMutex;              // rosbag2 writer
    std::unique_ptr<rosbag2_cpp::Writer> writer;

    void run()
    {
        while (true) {
            std::function<void()> task;
            {
                std::unique_lock<std::mutex> lock(mutex);
                wakeUp.wait(lock, [this]() { return stopping || !tasks.empty(); });
                if (tasks.empty()) {
                    return;     // Stopping, and everything is written
                }
                task = std::move(tasks.front());
                tasks.pop_front();
                active++;
            }
            try {
                task();
            } catch (const std::exception& e) {
                std::cerr << "Rosbag: " << e.what() << "\n";
                std::lock_guard<std::mutex> lock(mutex);
                droppedCount++;
            }
            {
                std::lock_guard<std::mutex> lock(mutex);
                active--;
            }
            idle.notify_all();
        }
    }

    void enqueue(std::function<void()> task)
    {
        {
            std::lock_guard<std::mutex> lock(mutex);
            if (!open) {
                return;
            }
            if (tasks.size() >= MAX_QUEUED) {
                droppedCount++;
                return;
            }
            tasks.push_back(std::move(task));
        }
        wakeUp.notify_one();
    }

    template<class Message>
    void write(const Message& message, const std::string& topic, uint64_t stampUs)
    {
        std::lock_guard<std::mutex> lock(writeMutex);
        if (writer) {
            writer->write(message, topic, rclcpp::Time(static_cast<int64_t>(stampUs) * 1000, RCL_SYSTEM_TIME));
        }
    }
};

BagWriter::BagWriter()
    : impl(std::make_unique<Impl>())
{
}

BagWriter::~BagWriter()
{
    close();
}

bool BagWriter::isAvailable()
{
    return true;
}

bool BagWriter::hasGridMap()
{
#ifdef WITH_GRID_MAP
    return true;
#else
    return false;
#endif
}

bool BagWriter::open(const std::string& uri, std::string& error)
{
    close();
    // The message type support and the MCAP plugin are found through the ament index, which setup.bash would set up
    setenv("AMENT_PREFIX_PATH", ROS_PREFIX, 0);
    setenv("ROS_DISTRO", std::filesystem::path(ROS_PREFIX).filename().c_str(), 0);   // Stored in the bag metadata
    try {
        auto writer = std::make_unique<rosbag2_cpp::Writer>();
        rosbag2_storage::StorageOptions options;
        options.uri = uri;
        options.storage_id = "mcap";
        // zstd compression of the MCAP chunks: the grids are mostly empty and the point clouds compress well
        // (the PNG images hardly). Every MCAP reader (ros2 bag, Foxglove, rosbag2_py) decompresses it.
        options.storage_preset_profile = "zstd_fast";
        writer->open(options);
        std::lock_guard<std::mutex> writeLock(impl->writeMutex);
        impl->writer = std::move(writer);
    } catch (const std::exception& e) {
        error = std::string("Cannot create the rosbag ") + uri + ": " + e.what();
        return false;
    }

    std::lock_guard<std::mutex> lock(impl->mutex);
    impl->open = true;
    impl->stopping = false;
    impl->droppedCount = 0;
    impl->bagUri = uri;
    for (size_t i = 0; i < THREADS; ++i) {
        impl->threads.emplace_back(&Impl::run, impl.get());
    }
    return true;
}

void BagWriter::close()
{
    {
        std::unique_lock<std::mutex> lock(impl->mutex);
        if (!impl->open) {
            return;
        }
        impl->open = false;     // No new messages
        impl->idle.wait(lock, [this]() { return impl->tasks.empty() && impl->active == 0; });
        impl->stopping = true;
    }
    impl->wakeUp.notify_all();
    for (std::thread& thread : impl->threads) {
        thread.join();
    }
    impl->threads.clear();
    std::lock_guard<std::mutex> writeLock(impl->writeMutex);
    impl->writer.reset();       // Writes the index and metadata.yaml
}

bool BagWriter::isOpen() const
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->open;
}

std::string BagWriter::uri() const
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->bagUri;
}

size_t BagWriter::pending() const
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->tasks.size() + impl->active;
}

size_t BagWriter::dropped() const
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->droppedCount;
}

void BagWriter::writeCalibration(uint64_t stampUs, const std::vector<CameraCalib>& calibs)
{
    {
        std::lock_guard<std::mutex> lock(impl->mutex);
        impl->calibs = calibs;
    }
    impl->enqueue([this, stampUs, calibs]() {
        tf2_msgs::msg::TFMessage tf;
        for (size_t i = 0; i < calibs.size(); ++i) {
            if (!calibs[i].valid) {
                continue;
            }
            // tLidarCam maps lidar points into the camera frame; the camera pose in the lidar frame is its inverse
            const Eigen::Isometry3d camInLidar(calibs[i].tLidarCam.inverse());
            geometry_msgs::msg::TransformStamped t;
            t.header.stamp = toStamp(stampUs);
            t.header.frame_id = LIDAR_FRAME;
            t.child_frame_id = cameraName(i, calibs);
            t.transform.translation.x = camInLidar.translation().x();
            t.transform.translation.y = camInLidar.translation().y();
            t.transform.translation.z = camInLidar.translation().z();
            t.transform.rotation = toQuaternion(Eigen::Quaterniond(camInLidar.rotation()));
            tf.transforms.push_back(t);
        }
        impl->write(tf, "/tf_static", stampUs);
    });
}

void BagWriter::writeClassNames(uint64_t stampUs, const std::vector<std::string>& names)
{
    impl->enqueue([this, stampUs, names]() {
        std::string json = "{";
        for (size_t k = 0; k < names.size(); ++k) {
            std::string name;
            for (const char c : names[k]) {
                if (c == '"' || c == '\\') {
                    name += '\\';
                }
                name += c;
            }
            json += (k ? ", \"" : "\"") + std::to_string(k) + "\": \"" + name + "\"";
        }
        json += "}";
        std_msgs::msg::String msg;
        msg.data = json;
        impl->write(msg, "/occupancy_map/class_names", stampUs);
    });
}

void BagWriter::writeScan(uint64_t stampUs, std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud, const GroundPlane& plane,
                          const OccupancyMapConfig& config, bool withGridMap)
{
    if (!cloud) {
        return;
    }
    impl->enqueue([this, stampUs, cloud]() {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = toStamp(stampUs);
        msg.header.frame_id = LIDAR_FRAME;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(cloud->size());
        msg.fields = {field("x", 0, sensor_msgs::msg::PointField::FLOAT32), field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
                      field("z", 8, sensor_msgs::msg::PointField::FLOAT32)};
        msg.is_bigendian = false;
        msg.point_step = 12;
        msg.row_step = msg.point_step * msg.width;
        msg.is_dense = true;
        msg.data.resize(static_cast<size_t>(msg.row_step));
        for (size_t i = 0; i < cloud->size(); ++i) {
            const float xyz[3] = {(*cloud)[i].position.x(), (*cloud)[i].position.y(), (*cloud)[i].position.z()};
            std::memcpy(msg.data.data() + i * msg.point_step, xyz, sizeof(xyz));
        }
        impl->write(msg, "/lidar/points", stampUs);
    });
    if (!plane.valid) {
        return;     // No ground: no occupancy map
    }
    impl->enqueue([this, stampUs, cloud, plane, config, withGridMap]() {
        tf2_msgs::msg::TFMessage tf;
        tf.transforms.push_back(groundTransform(stampUs, plane));
        impl->write(tf, "/tf", stampUs);

        const OccupancyMap map = buildOccupancyMap(projectToGround(*cloud, plane), config, false);
        impl->write(toOccupancyGrid(stampUs, map), "/occupancy_map", stampUs);
#ifdef WITH_GRID_MAP
        if (withGridMap) {
            impl->write(toGridMap(stampUs, map), "/occupancy_map/grid_map", stampUs);
        }
#else
        (void)withGridMap;
#endif
    });
}

void BagWriter::writeSegmentedScan(uint64_t stampUs, std::shared_ptr<const std::vector<vaiop::QtPoint>> cloud,
                                   std::vector<uint8_t> labels, const GroundPlane& plane, const OccupancyMapConfig& config)
{
    if (!cloud || labels.size() != cloud->size()) {
        return;
    }
    auto sharedLabels = std::make_shared<const std::vector<uint8_t>>(std::move(labels));
    impl->enqueue([this, stampUs, cloud, sharedLabels]() {
        sensor_msgs::msg::PointCloud2 msg;
        msg.header.stamp = toStamp(stampUs);
        msg.header.frame_id = LIDAR_FRAME;
        msg.height = 1;
        msg.width = static_cast<uint32_t>(cloud->size());
        // rgb: packed 0x00RRGGBB in a float, as PCL and RViz expect
        msg.fields = {field("x", 0, sensor_msgs::msg::PointField::FLOAT32), field("y", 4, sensor_msgs::msg::PointField::FLOAT32),
                      field("z", 8, sensor_msgs::msg::PointField::FLOAT32), field("rgb", 12, sensor_msgs::msg::PointField::FLOAT32),
                      field("label", 16, sensor_msgs::msg::PointField::UINT8)};
        msg.is_bigendian = false;
        msg.point_step = 20;
        msg.row_step = msg.point_step * msg.width;
        msg.is_dense = true;
        msg.data.assign(static_cast<size_t>(msg.row_step), 0);
        for (size_t i = 0; i < cloud->size(); ++i) {
            const vaiop::QtPoint& p = (*cloud)[i];
            const float xyz[3] = {p.position.x(), p.position.y(), p.position.z()};
            auto channel = [](float c) { return static_cast<uint32_t>(std::clamp(c, 0.0f, 1.0f) * 255.0f + 0.5f); };
            const uint32_t rgb = (channel(p.color.x()) << 16) | (channel(p.color.y()) << 8) | channel(p.color.z());
            uint8_t* out = msg.data.data() + i * msg.point_step;
            std::memcpy(out, xyz, sizeof(xyz));
            std::memcpy(out + 12, &rgb, sizeof(rgb));
            out[16] = (*sharedLabels)[i];
        }
        impl->write(msg, "/lidar/points_segmented", stampUs);
    });
#ifdef WITH_GRID_MAP
    if (plane.valid) {
        impl->enqueue([this, stampUs, cloud, sharedLabels, plane, config]() {
            const OccupancyMap map = buildOccupancyMap(projectToGround(*cloud, plane, sharedLabels.get()), config, true);
            impl->write(toGridMap(stampUs, map), "/occupancy_map/grid_map", stampUs);
        });
    }
#else
    (void)plane;
    (void)config;
#endif
}

void BagWriter::writeImage(size_t camera, uint64_t stampUs, const cv::Mat& imageRGB)
{
    if (imageRGB.empty()) {
        return;
    }
    // The camera frames are not changed after capture (a new cv::Mat per frame): sharing the data is safe
    impl->enqueue([this, camera, stampUs, imageRGB]() {
        CameraCalib calib;
        std::string name;
        {
            std::lock_guard<std::mutex> lock(impl->mutex);
            if (camera < impl->calibs.size()) {
                calib = impl->calibs[camera];
            }
            name = cameraName(camera, impl->calibs);
        }
        sensor_msgs::msg::CompressedImage image;
        image.header.stamp = toStamp(stampUs);
        image.header.frame_id = name;
        image.format = "bgr8; png compressed bgr8";
        cv::Mat bgr;
        cv::cvtColor(imageRGB, bgr, cv::COLOR_RGB2BGR);
        cv::imencode(".png", bgr, image.data, {cv::IMWRITE_PNG_COMPRESSION, 1});
        impl->write(image, "/" + name + "/image/compressed", stampUs);

        if (!calib.valid) {
            return;
        }
        sensor_msgs::msg::CameraInfo info;
        info.header = image.header;
        info.width = static_cast<uint32_t>(calib.imageSize.width > 0 ? calib.imageSize.width : imageRGB.cols);
        info.height = static_cast<uint32_t>(calib.imageSize.height > 0 ? calib.imageSize.height : imageRGB.rows);
        info.distortion_model = "plumb_bob";
        cv::Mat dist;
        calib.distCoeffs.convertTo(dist, CV_64F);
        info.d.assign(dist.begin<double>(), dist.end<double>());
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                info.k[r * 3 + c] = calib.K(r, c);
                info.p[r * 4 + c] = calib.K(r, c);
            }
        }
        info.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
        impl->write(info, "/" + name + "/camera_info", stampUs);
    });
}

void BagWriter::writeRobot(uint64_t stampUs, const RobotMetaData& robot)
{
    if (!robot.valid) {
        return;
    }
    impl->enqueue([this, stampUs, robot]() {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = toStamp(stampUs);
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = robot.x;
        odom.pose.pose.position.y = robot.y;
        odom.pose.pose.position.z = robot.z;
        // Roll, pitch, yaw in radians (ZYX)
        const Eigen::Quaterniond q = Eigen::AngleAxisd(robot.yaw, Eigen::Vector3d::UnitZ())
                                   * Eigen::AngleAxisd(robot.pitch, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(robot.roll, Eigen::Vector3d::UnitX());
        odom.pose.pose.orientation = toQuaternion(q);
        odom.twist.twist.linear.x = robot.odomLngSpeed;
        odom.twist.twist.angular.z = robot.odomAngSpeed;
        impl->write(odom, "/robot/odom", stampUs);

        sensor_msgs::msg::NavSatFix fix;
        fix.header.stamp = odom.header.stamp;
        fix.header.frame_id = "gnss";
        fix.latitude = robot.lat;
        fix.longitude = robot.lon;
        fix.altitude = std::numeric_limits<double>::quiet_NaN();
        fix.status.status = robot.gpsFix > 0 ? sensor_msgs::msg::NavSatStatus::STATUS_FIX
                                              : sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX;
        fix.status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
        fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_UNKNOWN;
        impl->write(fix, "/robot/gnss", stampUs);
    });
}

#else   // Without ROS 2

struct BagWriter::Impl {};

BagWriter::BagWriter() : impl(std::make_unique<Impl>()) {}
BagWriter::~BagWriter() = default;
bool BagWriter::isAvailable() { return false; }
bool BagWriter::hasGridMap() { return false; }
bool BagWriter::open(const std::string&, std::string& error)
{
    error = "Built without ROS 2 (/opt/ros/jazzy): recording to a rosbag is not available";
    return false;
}
void BagWriter::close() {}
bool BagWriter::isOpen() const { return false; }
std::string BagWriter::uri() const { return {}; }
size_t BagWriter::pending() const { return 0; }
size_t BagWriter::dropped() const { return 0; }
void BagWriter::writeCalibration(uint64_t, const std::vector<CameraCalib>&) {}
void BagWriter::writeClassNames(uint64_t, const std::vector<std::string>&) {}
void BagWriter::writeScan(uint64_t, std::shared_ptr<const std::vector<vaiop::QtPoint>>, const GroundPlane&, const OccupancyMapConfig&, bool) {}
void BagWriter::writeSegmentedScan(uint64_t, std::shared_ptr<const std::vector<vaiop::QtPoint>>, std::vector<uint8_t>,
                                   const GroundPlane&, const OccupancyMapConfig&) {}
void BagWriter::writeImage(size_t, uint64_t, const cv::Mat&) {}
void BagWriter::writeRobot(uint64_t, const RobotMetaData&) {}

#endif
