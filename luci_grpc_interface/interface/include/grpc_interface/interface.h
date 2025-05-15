/**
 * @file interface.h
 * 
 * @brief The public interface that is exposed to ROS2
 *
 * @copyright Copyright 2025 LUCI Mobility, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
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
#include <std_srvs/srv/empty.hpp>
#include <tf2_ros/transform_broadcaster.h>

// LUCI ROS2 libraries
#include <luci_messages/msg/luci_camera_info.hpp>
#include <luci_messages/msg/luci_encoders.hpp>
#include <luci_messages/msg/luci_imu.hpp>
#include <luci_messages/msg/luci_joystick.hpp>
#include <luci_messages/msg/luci_joystick_scaling.hpp>
#include <luci_messages/msg/luci_zone_scaling.hpp>

// How many messages a ROS topic should queue
constexpr size_t QUEUE_SIZE = 1;

/// Width of raw camera frames.
constexpr auto WIDTH = 640;

/// Height of raw camera frames.
constexpr auto HEIGHT = 360;

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
        std::shared_ptr<Luci::ROS2::DataBuffer<ImuData>> imuDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<EncoderData>> encoderDataBuff,
        std::shared_ptr<Luci::ROS2::DataBuffer<CameraIrData>> irDataBuffLeft,
        std::shared_ptr<Luci::ROS2::DataBuffer<CameraIrData>> irDataBuffRight,
        std::shared_ptr<Luci::ROS2::DataBuffer<CameraIrData>> irDataBuffRear, int initialFrameRate)
        : Node("interface"), luciInterface(luciInterface), cameraDataBuff(cameraDataBuff),
          radarDataBuff(radarDataBuff), ultrasonicDataBuff(ultrasonicDataBuff),
          joystickDataBuff(joystickDataBuff), zoneScalingDataBuff(zoneScalingDataBuff),
          joystickScalingDataBuff(joystickScalingDataBuff), ahrsInfoDataBuff(ahrsInfoDataBuff),
          imuDataBuff(imuDataBuff), encoderDataBuff(encoderDataBuff),
          irDataBuffLeft(irDataBuffLeft), irDataBuffRight(irDataBuffRight),
          irDataBuffRear(irDataBuffRear), initialFrameRate(initialFrameRate)
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

        this->imuPublisher =
            this->create_publisher<luci_messages::msg::LuciImu>("luci/imu", QUEUE_SIZE);

        this->encoderPublisher =
            this->create_publisher<luci_messages::msg::LuciEncoders>("luci/encoders", QUEUE_SIZE);

        /// ROS subscribers (takes data sent to it from other ROS nodes and sends it to LUCI over
        /// gRPC)
        this->remote_js_subscription_ = this->create_subscription<luci_messages::msg::LuciJoystick>(
            "luci/remote_joystick", QUEUE_SIZE,
            [this](luci_messages::msg::LuciJoystick::SharedPtr msg) { this->sendJsCallback(msg); });

        this->set_shared_remote_input_service = this->create_service<std_srvs::srv::Empty>(
            "luci/set_shared_remote_input",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                RCLCPP_INFO(rclcpp::get_logger("luci_interface"), "Set remote input service called");
                this->setSharedRemoteInputSource();
            });

        this->remove_shared_remote_input_service = this->create_service<std_srvs::srv::Empty>(
            "luci/remove_shared_remote_input",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                RCLCPP_INFO(rclcpp::get_logger("luci_interface"), "Remove remote input service called");
                this->removeSharedRemoteInputSource();
            });

        this->set_auto_remote_input_service = this->create_service<std_srvs::srv::Empty>(
            "luci/set_auto_remote_input",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                RCLCPP_INFO(rclcpp::get_logger("luci_interface"), "Set auto remote input service called");
                this->setAutoRemoteInputSource();
            });

        this->remove_auto_remote_input_service = this->create_service<std_srvs::srv::Empty>(
            "luci/remove_auto_remote_input",
            [this](const std::shared_ptr<rmw_request_id_t> request_header,
                   const std::shared_ptr<std_srvs::srv::Empty::Request> request,
                   const std::shared_ptr<std_srvs::srv::Empty::Response> response)
            {
                RCLCPP_INFO(rclcpp::get_logger("luci_interface"), "Remove auto remote input service called");
                this->removeAutoRemoteInputSource();
            });

        this->zoneScalingPublisher =
            this->create_publisher<luci_messages::msg::LuciZoneScaling>("luci/scaling", QUEUE_SIZE);

        this->joystickScalingPublisher =
            this->create_publisher<luci_messages::msg::LuciJoystickScaling>("luci/joystick_scaling",
                                                                            QUEUE_SIZE);

        this->joystickPositionPublisher = this->create_publisher<luci_messages::msg::LuciJoystick>(
            "luci/joystick_position", QUEUE_SIZE);

        // get the camera frame rate from the gRPC interface
        if (this->initialFrameRate != 0)
        {
            this->irLeftPublisher =
                this->create_publisher<sensor_msgs::msg::Image>("luci/ir_left_camera", QUEUE_SIZE);

            this->irRightPublisher =
                this->create_publisher<sensor_msgs::msg::Image>("luci/ir_right_camera", QUEUE_SIZE);

            this->irRearPublisher =
                this->create_publisher<sensor_msgs::msg::Image>("luci/ir_rear_camera", QUEUE_SIZE);

            this->leftCameraInfoPublisher =
                this->create_publisher<luci_messages::msg::LuciCameraInfo>("luci/left_camera_info",
                                                                           QUEUE_SIZE);

            this->rightCameraInfoPublisher =
                this->create_publisher<luci_messages::msg::LuciCameraInfo>("luci/right_camera_info",
                                                                           QUEUE_SIZE);

            this->rearCameraInfoPublisher =
                this->create_publisher<luci_messages::msg::LuciCameraInfo>("luci/rear_camera_info",
                                                                           QUEUE_SIZE);
        }

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
        grpcConverters.emplace_back(&Interface::processEncoderData, this);
        grpcConverters.emplace_back(&Interface::processLeftIrData, this);
        grpcConverters.emplace_back(&Interface::processRightIrData, this);
        grpcConverters.emplace_back(&Interface::processRearIrData, this);
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
    rclcpp::Publisher<luci_messages::msg::LuciEncoders>::SharedPtr encoderPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr irLeftPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr irRightPublisher;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr irRearPublisher;

    rclcpp::Publisher<luci_messages::msg::LuciCameraInfo>::SharedPtr leftCameraInfoPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciCameraInfo>::SharedPtr rightCameraInfoPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciCameraInfo>::SharedPtr rearCameraInfoPublisher;

    /// Shared pointers to subscribers (convention in ROS2)
    rclcpp::Subscription<luci_messages::msg::LuciJoystick>::SharedPtr remote_js_subscription_;

    /// Shared pointers to services (convention in ROS2)
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_shared_remote_input_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr remove_shared_remote_input_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr set_auto_remote_input_service;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr remove_auto_remote_input_service;

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
    std::shared_ptr<Luci::ROS2::DataBuffer<EncoderData>> encoderDataBuff;
    std::shared_ptr<Luci::ROS2::DataBuffer<CameraIrData>> irDataBuffLeft;
    std::shared_ptr<Luci::ROS2::DataBuffer<CameraIrData>> irDataBuffRight;
    std::shared_ptr<Luci::ROS2::DataBuffer<CameraIrData>> irDataBuffRear;
    int initialFrameRate;

    /// Functions to handle each unique data type and convert (each are ran on independent threads)
    void processCameraData();
    void processRadarData();
    void processUltrasonicData();
    void processZoneScalingData();
    void processJoystickScalingData();
    void processJoystickPositionData();
    void processAhrsData();
    void processImuData();
    void processEncoderData();
    void processLeftIrData();
    void processRightIrData();
    void processRearIrData();

    /// Update the IR frame rate
    void updateIrFrameRate(int rate);

    /// Subscriber callback ran in main thread
    void sendJsCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg);

    /// Service Callbacks ran in main thread for setting and removing the Shared and Autonomous remote input source
    void setSharedRemoteInputSource();
    void removeSharedRemoteInputSource();
    void setAutoRemoteInputSource();
    void removeAutoRemoteInputSource();
};
