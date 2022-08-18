/**
 * @file client.h
 *
 * The main connection point to the Luci safety system
 *
 * @copyright Copyright (c) 2022 Patroness, LLC. All Rights Reserved.
 */

#pragma once

// #include "ramp-assist/camera_types.h"
#include "data_buffer.h"
// #include "ramp-assist/js_types.h"
// #include "ramp-assist/ramp_info.h"
// #include "ramp-assist/ramp_landmark.h"
// #include "ramp-assist/start_mode_types.h"

// #include "ahrs_processing/ahrs_types.h"

// #include "tag-detection/pose.h"

// #include "edge_serial_driver/timestamp.h"
// #include "sensors_grpc/sensors.grpc.pb.h"

#include "../generated_code/client_library/ptolemy.grpc.pb.h"

// #include "luci_config/ramp_config.h"

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

// /// Time (seconds) to timeout on the physical js stream read
// constexpr auto JOYSTICK_READ_TIMEOUT = std::chrono::seconds(5);

// // Depending on ramp mode status, process only a certain amount of frames per second
// constexpr int FRAME_RATE_RAMP_MODE_ON = 2;   // ~ 7 fps
// constexpr int FRAME_RATE_RAMP_MODE_OFF = 15; // 1 fps

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

        // pcl::PointCloud<pcl::PointXYZ> cameraPointCloud;
        // pcl::PointCloud<pcl::PointXYZ> ultrasonicPointCloud;
        // pcl::PointCloud<pcl::PointXYZ> radarPointCloud;
        std::shared_ptr<grpc::Channel> channel,

        std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
        std::shared_ptr<DataBuffer<float>> chairSpeedDataBuff)

        // std::shared_ptr<DataBuffer<Luci::Config::JoystickOption>> joystickOptionDataBuff,
        // std::shared_ptr<DataBuffer<SystemJoystick>> virtualJoystickDataBuff,
        // std::shared_ptr<DataBuffer<SystemJoystick>> physicalJoystickDataBuff,
        // std::shared_ptr<DataBuffer<Luci::RampAssist::Meta::Info>> rampInfoDataBuff,
        // std::shared_ptr<DataBuffer<Luci::RampAssist::Landmark::RampLandmark>>
        // rampLandmarkDataBuff)
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

    // std::shared_ptr<DataBuffer<CameraFrame>> irDataBuffLeft;
    // std::shared_ptr<DataBuffer<CameraFrame>> irDataBuffRight;
    std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff;
    // std::shared_ptr<DataBuffer<Luci::AhrsData>> ahrsDataBuff;
    // std::shared_ptr<DataBuffer<int>> pressCountDataBuff;
    // std::shared_ptr<DataBuffer<bool>> luciOverrideDataBuff;
    // std::shared_ptr<DataBuffer<Luci::StartModes>> startModeDataBuff;
    // std::shared_ptr<DataBuffer<Luci::Config::JoystickOption>> joystickOptionDataBuff;
    // std::shared_ptr<DataBuffer<SystemJoystick>> virtualJoystickDataBuff;
    // std::shared_ptr<DataBuffer<SystemJoystick>> physicalJoystickDataBuff;

    // std::shared_ptr<DataBuffer<Luci::RampAssist::Meta::Info>> rampInfoDataBuff;
    // std::shared_ptr<DataBuffer<Luci::RampAssist::Landmark::RampLandmark>> rampLandmarkDataBuff;

    // /// Ramp state tracker
    // std::atomic<bool> rampActive = false;
    // std::atomic<bool> approachActive = false;

    /// Time trackers to ensure safe time between remote js sends
    // std::chrono::time_point<std::chrono::high_resolution_clock> lastJSTime =
    //     std::chrono::high_resolution_clock::now();

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

    // Helpers

    /**
     * @brief Calculate the timestamp of a rpc response into usable format
     *
     * @tparam T
     * @param response
     * @return Luci::Timing::Timestamp
     */
    // template <typename T> Luci::Timing::Timestamp calculateTimeStamp(T response) const
    // {
    //     unsigned long timeNanoSeconds =
    //         (response->timestamp().seconds() * 1000000000) + (response->timestamp().nanos());
    //     auto timestamp =
    //         std::chrono::time_point<std::chrono::steady_clock, std::chrono::nanoseconds>(
    //             std::chrono::nanoseconds(timeNanoSeconds));
    //     return timestamp;
    // }

    void readRadarPointData() const;
    void readCameraPointData() const;
    void readUltrasonicData() const;
    void readChairSpeedData() const;

  private:
    /// Threads for each endpoint
    std::vector<std::thread> grpcThreads;
};
} // namespace Luci::ROS2
