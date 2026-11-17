#pragma once

// Conversions to ROS 2 messages, shared by the rosbag recording (BagWriter) and the live topics (RosPublisher).
// Only with ROS 2 (WITH_ROSBAG).
#ifdef WITH_ROSBAG

#include <cmath>
#include <cstdint>

#include <Eigen/Geometry>

#include <builtin_interfaces/msg/time.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/odometry.hpp>

#include <Utils/GroundPlane.h>
#include <Utils/MetaData.hpp>
#include <Utils/OccupancyMap.h>

namespace rosmsg {
    inline constexpr const char* LIDAR_FRAME = "lidar";
    inline constexpr const char* GROUND_FRAME = "ground";

    inline builtin_interfaces::msg::Time toStamp(uint64_t us)
    {
        builtin_interfaces::msg::Time t;
        t.sec = static_cast<int32_t>(us / 1000000);
        t.nanosec = static_cast<uint32_t>((us % 1000000) * 1000);
        return t;
    }

    inline geometry_msgs::msg::Quaternion toQuaternion(const Eigen::Quaterniond& q)
    {
        geometry_msgs::msg::Quaternion m;
        m.x = q.x();
        m.y = q.y();
        m.z = q.z();
        m.w = q.w();
        return m;
    }

    // ground -> lidar: the rows of the rotation are the ground axes (forward, left, up) in the lidar frame
    inline geometry_msgs::msg::TransformStamped groundTransform(uint64_t stampUs, const GroundPlane& plane)
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

    inline nav_msgs::msg::OccupancyGrid toOccupancyGrid(uint64_t stampUs, const OccupancyMap& map)
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

    // odom -> base_link from the robot state (Redis). The angles of gps.ref.state are in degrees.
    inline nav_msgs::msg::Odometry toOdometry(uint64_t stampUs, const RobotMetaData& robot)
    {
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = toStamp(stampUs);
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";
        odom.pose.pose.position.x = robot.x;
        odom.pose.pose.position.y = robot.y;
        odom.pose.pose.position.z = robot.z;
        constexpr double DEG = M_PI / 180.0;
        // Roll, pitch, yaw (ZYX)
        const Eigen::Quaterniond q = Eigen::AngleAxisd(robot.yaw * DEG, Eigen::Vector3d::UnitZ())
                                   * Eigen::AngleAxisd(robot.pitch * DEG, Eigen::Vector3d::UnitY())
                                   * Eigen::AngleAxisd(robot.roll * DEG, Eigen::Vector3d::UnitX());
        odom.pose.pose.orientation = toQuaternion(q);
        odom.twist.twist.linear.x = robot.odomLngSpeed;     // [m/s]
        odom.twist.twist.angular.z = robot.odomAngSpeed;    // [rad/s]
        return odom;
    }
}

#endif
