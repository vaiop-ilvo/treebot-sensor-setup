#include <Utils/RosPublisher.h>

#ifdef WITH_ROSBAG
#include <cstdlib>
#include <filesystem>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <Utils/RosMessages.h>

struct RosPublisher::Impl {
    std::mutex mutex;
    RosConfig config;
    rclcpp::Context::SharedPtr context;
    rclcpp::Node::SharedPtr node;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy;
    rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr tf;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr classes;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr classNames;
    std::string classNamesJson;         // Published again when the node (re)starts

    void publishClassNames()
    {
        if (classNames && !classNamesJson.empty()) {
            std_msgs::msg::String msg;
            msg.data = classNamesJson;
            classNames->publish(msg);
        }
    }
};

RosPublisher::RosPublisher()
    : impl(std::make_unique<Impl>())
{
}

RosPublisher::~RosPublisher()
{
    stop();
}

bool RosPublisher::isAvailable()
{
    return true;
}

bool RosPublisher::start(const RosConfig& config, std::string& error)
{
    stop();
    // The RMW implementation and the message type support are found through the ament index (as setup.bash would)
    setenv("AMENT_PREFIX_PATH", ROS_PREFIX, 0);
    setenv("ROS_DISTRO", std::filesystem::path(ROS_PREFIX).filename().c_str(), 0);
    try {
        // Own context, without signal handlers: Ctrl+C stays with the application
        auto context = std::make_shared<rclcpp::Context>();
        context->init(0, nullptr, rclcpp::InitOptions());
        rclcpp::NodeOptions options;
        options.context(context);
        auto node = std::make_shared<rclcpp::Node>(config.nodeName, options);

        // The newest map is all that matters: keep one, reliable (RViz and rclpy defaults)
        const rclcpp::QoS latest = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
        std::lock_guard<std::mutex> lock(impl->mutex);
        impl->config = config;
        impl->occupancy = node->create_publisher<nav_msgs::msg::OccupancyGrid>(config.occupancyTopic, latest);
        impl->tf = node->create_publisher<tf2_msgs::msg::TFMessage>("/tf", rclcpp::QoS(10));
        impl->odom = node->create_publisher<nav_msgs::msg::Odometry>(config.odomTopic, rclcpp::QoS(10));
        impl->classes = node->create_publisher<nav_msgs::msg::OccupancyGrid>(config.classTopic, latest);
        // Latched: a node that starts later still gets the names
        impl->classNames = node->create_publisher<std_msgs::msg::String>(
            config.classTopic.substr(0, config.classTopic.rfind('/')) + "/class_names",
            rclcpp::QoS(rclcpp::KeepLast(1)).reliable().transient_local());
        impl->node = node;
        impl->context = context;
        impl->publishClassNames();
    } catch (const std::exception& e) {
        error = std::string("Cannot start the ROS 2 node: ") + e.what();
        return false;
    }
    return true;
}

void RosPublisher::stop()
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    impl->occupancy.reset();
    impl->tf.reset();
    impl->odom.reset();
    impl->classes.reset();
    impl->classNames.reset();
    impl->node.reset();
    if (impl->context) {
        impl->context->shutdown("stopped");
        impl->context.reset();
    }
}

bool RosPublisher::isRunning() const
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    return impl->node != nullptr;
}

void RosPublisher::publish(const ProcessedFrame& frame)
{
    if (!frame.frame) {
        return;
    }
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (!impl->node) {
        return;
    }
    const uint64_t stamp = frame.frame->triggerTs;
    try {
        // The robot state first: a node that receives the map can then look up the pose of that same moment
        if (frame.frame->robot.valid && impl->config.publishOdom) {
            impl->odom->publish(rosmsg::toOdometry(stamp, frame.frame->robot));
        }
        if (frame.plane.valid) {
            tf2_msgs::msg::TFMessage tf;
            tf.transforms.push_back(rosmsg::groundTransform(stamp, frame.plane));
            impl->tf->publish(tf);
        }
        if (frame.occupancy && impl->config.publishOccupancy) {
            impl->occupancy->publish(rosmsg::toOccupancyGrid(stamp, *frame.occupancy));
        }
    } catch (const std::exception&) {
        // Context shut down while publishing: the next frames are dropped by the check above
    }
}

void RosPublisher::publishClasses(uint64_t stampUs, const OccupancyMap& classMap)
{
    std::lock_guard<std::mutex> lock(impl->mutex);
    if (!impl->node || !impl->config.publishClasses || !classMap.hasClasses) {
        return;
    }
    nav_msgs::msg::OccupancyGrid grid = rosmsg::toOccupancyGrid(stampUs, classMap);
    for (size_t cell = 0; cell < grid.data.size(); ++cell) {
        const uint8_t label = classMap.classes[cell];
        const bool classified = classMap.occupancy[cell] == OccupancyMap::OCCUPIED && label != BirdEyePoint::NO_LABEL
                                && label <= 127;   // int8 data
        grid.data[cell] = classified ? static_cast<int8_t>(label) : int8_t(-1);
    }
    try {
        impl->classes->publish(grid);
    } catch (const std::exception&) {
    }
}

void RosPublisher::setClassNames(const std::vector<std::string>& names)
{
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
    std::lock_guard<std::mutex> lock(impl->mutex);
    impl->classNamesJson = json;
    try {
        impl->publishClassNames();
    } catch (const std::exception&) {
    }
}

#else   // Without ROS 2

struct RosPublisher::Impl {};

RosPublisher::RosPublisher() : impl(std::make_unique<Impl>()) {}
RosPublisher::~RosPublisher() = default;
bool RosPublisher::isAvailable() { return false; }
bool RosPublisher::start(const RosConfig&, std::string& error)
{
    error = "Built without ROS 2 (/opt/ros/jazzy): publishing on ROS topics is not available";
    return false;
}
void RosPublisher::stop() {}
bool RosPublisher::isRunning() const { return false; }
void RosPublisher::publish(const ProcessedFrame&) {}
void RosPublisher::publishClasses(uint64_t, const OccupancyMap&) {}
void RosPublisher::setClassNames(const std::vector<std::string>&) {}

#endif
