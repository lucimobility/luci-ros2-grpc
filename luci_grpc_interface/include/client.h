/**
 * @file client.h
 *
 * The main connection point to the Luci safety system
 *
 * @copyright Copyright (c) 2022 Patroness, LLC. All Rights Reserved.
 */

#pragma once

#include "data_buffer.h"

#include "../generated_code/client_library/ptolemy.grpc.pb.h"

#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

#include <opencv2/opencv.hpp>

#include <spdlog/spdlog.h>

#include <chrono>
#include <condition_variable>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

using sensors::CameraPoints;
using sensors::ChairSpeed;
using sensors::RadarPoints;
using sensors::UltrasonicDistances;

struct SystemJoystick
{
    int forwardBack;
    int leftRight;

    SystemJoystick(int forwardBack, int leftRight) : forwardBack(forwardBack), leftRight(leftRight)
    {
    }
};

namespace Luci::ROS2
{

class ClientGuide
{
  public:
    /**
     * @brief Construct a new Client Guide object
     *
     * @param channel
     * @param irDataBuffLeft
     * @param irDataBuffRight
     * @param joystickDataBuff
     * @param ahrsDataBuff
     * @param pressCountDataBuff
     * @param luciOverrideDataBuff
     * @param startModeDataBuff
     * @param joystickOptionDataBuff
     */
    explicit ClientGuide(

        std::shared_ptr<grpc::Channel> channel,

        std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
        std::shared_ptr<DataBuffer<float>> chairSpeedDataBuff)

        ;

    /**
     * @brief Destroy the Client Guide object
     *
     */
    ~ClientGuide();

    /// Client stub
    std::unique_ptr<sensors::Sensors::Stub> stub_;

    std::chrono::time_point<std::chrono::high_resolution_clock> lastJSTime =
        std::chrono::high_resolution_clock::now();
    std::chrono::time_point<std::chrono::high_resolution_clock> lastFrameTime =
        std::chrono::high_resolution_clock::now();
    /// Data Buffers for the transmission of data to upper level modules
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff;
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff;
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff;
    std::shared_ptr<DataBuffer<float>> chairSpeedDataBuff;

    std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff;

    // Single calls over gRPC

    /**
     * @brief Interface to turn on engaged mode at the msp level
     *
     * @return bool success code (true-success, false-failure)
     */
    bool activateEngagedMode() const;

    /**
     * @brief Interface to turn on user mode at the msp level
     *
     * @return bool success code (true-success, false-failure)
     */
    bool activateUserMode() const;

    /**
     * @brief Interface to turn on auto mode at the msp level
     *
     * @return bool success code (true-success, false-failure)
     */
    bool activateAutoMode() const;

    /**
     * @brief Send JS values to luci sensors
     *
     * @param forwardBack
     * @param leftRight
     * @return int success code (0-success, 1-failure)
     */
    int sendJS(int forwardBack, int leftRight);

    // Readers

    /**
     * @brief Read the chairs joystick position
     *
     */
    void readJoystickPosition() const;

    void readRadarPointData() const;
    void readCameraPointData() const;
    void readUltrasonicData() const;
    void readChairSpeedData() const;

  private:
    /// Threads for each endpoint
    std::vector<std::thread> grpcThreads;
};
} // namespace Luci::ROS2
