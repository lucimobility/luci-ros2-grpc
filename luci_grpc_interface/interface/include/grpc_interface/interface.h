/**
 * @file interface.h
 * @brief The public interface that is exposed to ROS2
 * @date 2023-08-10
 *
 * @copyright Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.
 */

// TODO: clp If you change the QoS make sure you update the docs to say what subscriber calls are
// compatible

#pragma once

// System libraries
#include <iostream>
#include <memory>
#include <pcl_conversions/pcl_conversions.h>

// LUCI libraries
#include "client/client.h"
#include "client/common_types.h"
#include "client/data_buffer.h"

// ROS2 libraries
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

// LUCI ROS2 libraries
#include <luci_messages/msg/luci_drive_mode.hpp>
#include <luci_messages/msg/luci_imu.hpp>
#include <luci_messages/msg/luci_joystick.hpp>
#include <luci_messages/msg/luci_joystick_scaling.hpp>
#include <luci_messages/msg/luci_zone_scaling.hpp>

// How many messages a ROS topic should queue
constexpr size_t QUEUE_SIZE = 1;

/**
 * @brief The interface node that is publicly exposed to ROS2
 *
 * @note All subscribers and publishers should be established in the class construction, this node
 * is using a single thread static executor for performance so it needs to know about all
 * subscribers and publishers at compile time.
 *
 */
class Interface : public rclcpp::Node
{

  public:
    Interface(
        std::shared_ptr<Luci::ROS2::ClientGuide> luciInterface,
        std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<SystemJoystick>> joystickDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<LuciZoneScaling>> zoneScalingDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<AhrsInfo>> ahrsInfoDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<IMUData>> imuDataBuff)
        : Node("interface"), luciInterface(luciInterface), cameraDataBuff(cameraDataBuff),
          radarDataBuff(radarDataBuff), ultrasonicDataBuff(ultrasonicDataBuff),
          joystickDataBuff(joystickDataBuff), zoneScalingDataBuff(zoneScalingDataBuff),
          joystickScalingDataBuff(joystickScalingDataBuff), ahrsInfoDataBuff(ahrsInfoDataBuff),
          imuDataBuff(imuDataBuff)
    {
        /// ROS publishers (sends the LUCI gRPC data to ROS on the specified topic)
        this->cameraPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("luci/camera_points", QUEUE_SIZE);

        this->radarPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("luci/radar_points", QUEUE_SIZE);

        this->ultrasonicPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "luci/ultrasonic_points", QUEUE_SIZE);

        this->odomPublisher =
            this->create_publisher<nav_msgs::msg::Odometry>("luci/odom", QUEUE_SIZE);

        this->imuPublisher = this->create_publisher<luci_messages::msg::LuciImu>("luci/imu", 1);

        /// ROS subscribers (takes data sent to it from other ROS nodes and sends it to LUCI over
        /// gRPC)
        this->remote_js_subscription_ = this->create_subscription<luci_messages::msg::LuciJoystick>(
            "luci/remote_joystick", 1,
            [this](luci_messages::msg::LuciJoystick::SharedPtr msg) { this->sendJsCallback(msg); });

        this->luci_mode_switch_subscription_ =
            this->create_subscription<luci_messages::msg::LuciDriveMode>(
                "luci/drive_mode", 1,
                [this](luci_messages::msg::LuciDriveMode::SharedPtr msg)
                { this->switchLuciModeCallback(msg); });

        this->zoneScalingPublisher =
            this->create_publisher<luci_messages::msg::LuciZoneScaling>("luci/scaling", QUEUE_SIZE);

        this->joystickScalingPublisher =
            this->create_publisher<luci_messages::msg::LuciJoystickScaling>("luci/joystick_scaling",
                                                                            QUEUE_SIZE);

        this->joystickPositionPublisher = this->create_publisher<luci_messages::msg::LuciJoystick>(
            "luci/joystick_position", QUEUE_SIZE);

        // TODO: clp Should the processing just be handled in the gRPC threads?
        // Spin up a single thread for every gRPC <-> ROS translation
        grpcConverters.emplace_back(&Interface::processCameraData, this);
        grpcConverters.emplace_back(&Interface::processRadarData, this);
        grpcConverters.emplace_back(&Interface::processUltrasonicData, this);
        grpcConverters.emplace_back(&Interface::processZoneScalingData, this);
        grpcConverters.emplace_back(&Interface::processJoystickScalingData, this);
        grpcConverters.emplace_back(&Interface::processJoystickPositionData, this);
        grpcConverters.emplace_back(&Interface::processAhrsData, this);
        grpcConverters.emplace_back(&Interface::processImuData, this);
    }

    /// Destructor
    ~Interface() = default;

  private:
    /// Shared pointers to publishers (convention in ROS2)
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cameraPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radarPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ultrasonicPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciZoneScaling>::SharedPtr zoneScalingPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciJoystickScaling>::SharedPtr joystickScalingPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciJoystick>::SharedPtr joystickPositionPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciImu>::SharedPtr imuPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster =
        std::make_shared<tf2_ros::TransformBroadcaster>(this);

    /// Threads for each endpoint
    std::vector<std::thread> grpcConverters;

    // TODO: Add in data writer library to not need to pass in luciInterface as this is not a clean
    // dependency chain
    std::shared_ptr<Luci::ROS2::ClientGuide> luciInterface;
    std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<LuciZoneScaling>> zoneScalingDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<SystemJoystick>> joystickDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<AhrsInfo>> ahrsInfoDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<ImuData>> imuDataBuff;

    /// Shared pointers to subscribers (convention in ROS2)
    rclcpp::Subscription<luci_messages::msg::LuciJoystick>::SharedPtr remote_js_subscription_;
    rclcpp::Subscription<luci_messages::msg::LuciDriveMode>::SharedPtr
        luci_mode_switch_subscription_;

    /// Functions to handle each unique data type and convert (each are ran on independent threads)
    void processCameraData();
    void processRadarData();
    void processUltrasonicData();
    void processZoneScalingData();
    void processJoystickScalingData();
    void processJoystickPositionData();
    void processAhrsData();
    void processImuData();

    /// Subscriber callback ran in main thread
    void sendJsCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg);
    void switchLuciModeCallback(const luci_messages::msg::LuciDriveMode::SharedPtr msg);
};
