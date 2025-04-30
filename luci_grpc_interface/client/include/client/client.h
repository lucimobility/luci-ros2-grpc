/**
 * @file client.h
 *
 * The main connection point to the Luci safety system
 *
 * @copyright Copyright 2024 LUCI Mobility, Inc
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

#pragma once

// LUCI libraries
#include "common_types.h"
#include "data_buffer.h"
#include "sensors_grpc/sensors.grpc.pb.h"

// gRPC files
#include <grpc/grpc.h>
#include <grpcpp/grpcpp.h>
#include <grpcpp/security/server_credentials.h>
#include <grpcpp/server.h>
#include <grpcpp/server_builder.h>
#include <grpcpp/server_context.h>

// System libraries
#include <chrono>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <spdlog/spdlog.h>
#include <stdexcept>
#include <thread>
#include <vector>

using sensors::AhrsData;
using sensors::CameraMetaData;
using sensors::CameraPoints;
using sensors::JoystickData;
using sensors::NavigationScaling;
using sensors::RadarPoints;
using sensors::UltrasonicDistances;

namespace Luci::ROS2
{

/// Width of raw camera frames.
constexpr auto WIDTH = 640;

/// Height of raw camera frames.
constexpr auto HEIGHT = 360;

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
     * @param zoneScalingDataBuff
     * @param joystickScalingDataBuff
     * @param ahrsInfoBuff
     * @param imuDataBuff
     * @param encoderDataBuff
     * @param irDataBuffLeft
     * @param irDataBuffRight
     * @param irDataBuffRear
    * @param initialFrameRate

     */
    explicit ClientGuide(
        std::shared_ptr<grpc::Channel> channel,
        std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
        std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
        std::shared_ptr<DataBuffer<LuciZoneScaling>> zoneScalingDataBuff,
        std::shared_ptr<DataBuffer<SystemJoystick>> joystickScalingDataBuff,
        std::shared_ptr<DataBuffer<AhrsInfo>> ahrsInfoBuff,
        std::shared_ptr<DataBuffer<ImuData>> imuDataBuff,
        std::shared_ptr<DataBuffer<EncoderData>> encoderDataBuff,
        std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffLeft,
        std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRight,
        std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRear, int initialFrameRate);

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
    std::shared_ptr<DataBuffer<LuciZoneScaling>> zoneScalingDataBuff;
    std::shared_ptr<DataBuffer<SystemJoystick>> joystickScalingDataBuff;
    std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff;
    std::shared_ptr<DataBuffer<AhrsInfo>> ahrsDataBuff;
    std::shared_ptr<DataBuffer<ImuData>> imuDataBuff;
    std::shared_ptr<DataBuffer<EncoderData>> encoderDataBuff;
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffLeft;
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRight;
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRear;

    /// Data Buffer for thread safe updates to the IR frame a given client is getting sent
    DataBuffer<int> irFrameRateDataBuff;

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
     * @brief Set the input source for the chair to remote
     *
     */
    void setInputSource(InputSource source) const;

    /**
     * @brief Remove the input source for the chair remote
     *
     */
    void removeInputSource(InputSource source) const;

    /**
     * @brief Send JS values to luci sensors
     *
     * @param forwardBack
     * @param leftRight
     * @param source
     * @return int success code (0-success, 1-failure)
     */
    int sendJS(int forwardBack, int leftRight, InputSource source);

    /**
     * @brief Updates the IR frame rate while a stream is already active. This is used by clients
     * like ramp assist to change frame rate through out the applications life time.
     *
     * @param rate new rate in FPS (2 = 2 IR frames sent per second)
     * @note The rate selected is not guaranteed to be exactly what is received as the max fps is 15
     * requested rate that are not a clean multiple will be rounded.
     */
    void updateFrameRate(int rate);

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
     * @brief Read the chairs zone scaling data
     *
     */
    void readZoneScalingData() const;

    /**
     * @brief Read the chairs joystick scaling data
     *
     */
    void readJoystickScalingData() const;

    /**
     * @brief Read the chairs AHRS data
     *
     */
    void readAhrsData() const;

    /**
     * @brief Read chairs raw IMU data
     *
     */
    void readImuData() const;

    /**
     * @brief Read the chairs encoder data
     *
     */
    void readEncoderData() const;

    /**
     * @brief Read the IR frame from both front cameras.
     * To add additional cameras, make sure it is being streamed on luci-sensors.
     *
     * @param initialRate The rate used at client connection start. Defaults to 1.
     */
    void readIrFrame(int initialRate = 1);

  private:
    /// Threads for each endpoint
    std::vector<std::thread> grpcThreads;
};
} // namespace Luci::ROS2
