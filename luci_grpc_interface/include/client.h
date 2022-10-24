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
using sensors::JoystickData;
using sensors::NavigationScaling;
using sensors::RadarPoints;
using sensors::UltrasonicDistances;

struct SystemJoystick
{
    int forward_back;
    int left_right;

    SystemJoystick(int forward_back, int left_right)
        : forward_back(forward_back), left_right(left_right)
    {
    }
};

struct LuciJoystickScaling
{
    int forward_back;
    int left_right;
    std::string joystick_zone;

    LuciJoystickScaling(int forward_back, int left_right, std::string joystick_zone)
        : forward_back(forward_back), left_right(left_right), joystick_zone(joystick_zone)
    {
    }
};

struct LuciZoneScaling
{
    float front_fb;
    float front_rl;
    float front_right_fb;
    float front_right_rl;
    float front_left_fb;
    float front_left_rl;
    float right_fb;
    float right_rl;
    float left_fb;
    float left_rl;
    float back_right_fb;
    float back_right_rl;
    float back_left_fb;
    float back_left_rl;
    float back_fb;
    float back_rl;

    LuciZoneScaling(float front_fb, float front_rl, float front_right_fb, float front_right_rl,
                    float front_left_fb, float front_left_rl, float right_fb, float right_rl,
                    float left_fb, float left_rl, float back_right_fb, float back_right_rl,
                    float back_left_fb, float back_left_rl, float back_fb, float back_rl)
        : front_fb(front_fb), front_rl(front_rl), front_right_fb(front_right_fb),
          front_right_rl(front_right_rl), front_left_fb(front_left_fb),
          front_left_rl(front_left_rl), right_fb(right_fb), right_rl(right_rl), left_fb(left_fb),
          left_rl(left_rl), back_right_fb(back_right_fb), back_right_rl(back_right_rl),
          back_left_fb(back_left_fb), back_left_rl(back_left_rl), back_fb(back_fb), back_rl(back_rl)
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
     * @param joystickDataBuff
     * @param cameraDataBuff
     * @param radarDataBuff
     * @param ultrasonicDataBuff
     * @param chairSpeedDataBuff
     * @param zoneScalingDataBuff
     * @param joystickScalingDataBuff
     */
    explicit ClientGuide(
        std::shared_ptr<grpc::Channel> channel,
        std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
        std::shared_ptr<DataBuffer<float>> chairSpeedDataBuff,
        std::shared_ptr<DataBuffer<LuciZoneScaling>> zoneScalingDataBuff,
        std::shared_ptr<DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff);

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
    std::shared_ptr<DataBuffer<LuciZoneScaling>> zoneScalingDataBuff;
    std::shared_ptr<DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff;
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

    /**
     * @brief Read the chairs radar data
     *
     */
    void readRadarData() const;

    /**
     * @brief Read the chairs camera data
     *
     */
    void readCameraData() const;

    /**
     * @brief Read the chairs ultrasonic data
     *
     */
    void readUltrasonicData() const;

    /**
     * @brief Read the chairs speed data
     *
     */
    void readChairSpeedData() const;

    /**
     * @brief Read the chairs zone scaling data
     *
     */
    void readZoneScalingData() const;

    /**
     * @brief Read the chairs joystick scaling data
     *
     */
    void readJoystickScalingData() const;

  private:
    /// Threads for each endpoint
    std::vector<std::thread> grpcThreads;
};
} // namespace Luci::ROS2
