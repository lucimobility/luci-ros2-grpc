/**
 * @file main.cpp
 * @brief The main executable file for the interface node
 * @date 2023-08-10
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

#include "client/client.h"
#include "grpc_interface/interface.h"
#include "rclcpp/rclcpp.hpp"
#include "tclap/CmdLine.h"

#include <iostream>

/// Time (seconds) to timeout on attempt to connect to grpc
constexpr auto GRPC_CHANNEL_TIMEOUT = std::chrono::seconds(5);

/// Number of attempts allowed to connect to sensors before erroring
constexpr int GRPC_RETRIES = 1;

int main(int argc, char* argv[])
{

    std::string host;
    std::string port;

    // Setup command line parser
    TCLAP::CmdLine cmd("LUCI ROS2 to gRPC interface", ' ', "---");

    // Command to pass in chair ip address in gRPC mode
    TCLAP::ValueArg<std::string> hostArg("a", "host-address", "The host address. gRPC mode only.",
                                         true, "", "string");

    // Command to pass in chair port in gRPC mode
    TCLAP::ValueArg<std::string> portArg("p", "gRPC-port", "Port number. gRPC mode only.", false,
                                         "50051", "string");

    // Command to pass in camera frame rate in gRPC mode
    TCLAP::ValueArg<int> rateArg("f", "frame-rate", "IR Frame Rate gRPC mode only.", false, 0,
                                 "int");

    // Add the value arguments to the command line parser
    cmd.add(hostArg);
    cmd.add(portArg);
    cmd.add(rateArg);

    // Parse the argv array.
    cmd.parse(argc, argv);

    // Set the host and port values (if not set defaults used)
    host = hostArg.getValue();
    port = portArg.getValue();
    int frameRate = rateArg.getValue();

    // Initialize ROS stack and executor, Note: all argument parsing is handled externally by tclap
    rclcpp::init(0, nullptr);
    rclcpp::executors::SingleThreadedExecutor executor;

    // gRPC timeout
    auto deadline = std::chrono::system_clock::now() + GRPC_CHANNEL_TIMEOUT;

    // Data buff for passing around information
    auto joystickDataBuff = std::make_shared<Luci::ROS2::DataBuffer<SystemJoystick>>();

    auto zoneScalingDataBuff = std::make_shared<Luci::ROS2::DataBuffer<LuciZoneScaling>>();

    auto joystickScalingDataBuff = std::make_shared<Luci::ROS2::DataBuffer<SystemJoystick>>();

    auto ahrsInfoDataBuff = std::make_shared<Luci::ROS2::DataBuffer<AhrsInfo>>();

    auto imuDataBuff = std::make_shared<Luci::ROS2::DataBuffer<ImuData>>();

    auto encoderDataBuff = std::make_shared<Luci::ROS2::DataBuffer<EncoderData>>();

    auto cameraDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>>();

    auto radarDataBuff = std::make_shared<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>>();

    auto ultrasonicDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>>();

    auto irDataBuffLeft = std::make_shared<Luci::ROS2::DataBuffer<CameraIrData>>();

    auto irDataBuffRight = std::make_shared<Luci::ROS2::DataBuffer<CameraIrData>>();

    auto irDataBuffRear = std::make_shared<Luci::ROS2::DataBuffer<CameraIrData>>();

    auto grpcChannel =
        grpc::CreateChannel(fmt::format("{}:{}", host, port), grpc::InsecureChannelCredentials());
    bool connected = grpcChannel->WaitForConnected(deadline);
    if (!connected)
    {
        spdlog::error("grpc server NOT connected");
        return 1;
    }

    // gRPC connection to LUCI
    auto luciInterface = std::make_shared<Luci::ROS2::ClientGuide>(
        grpcChannel, joystickDataBuff, cameraDataBuff, radarDataBuff, ultrasonicDataBuff,
        zoneScalingDataBuff, joystickScalingDataBuff, ahrsInfoDataBuff, imuDataBuff,
        encoderDataBuff, irDataBuffLeft, irDataBuffRight, irDataBuffRear, frameRate);

    // // ROS connection
    auto interface_node = std::make_shared<Interface>(
        luciInterface, cameraDataBuff, radarDataBuff, ultrasonicDataBuff, joystickDataBuff,
        zoneScalingDataBuff, joystickScalingDataBuff, ahrsInfoDataBuff, imuDataBuff,
        encoderDataBuff, irDataBuffLeft, irDataBuffRight, irDataBuffRear);

    executor.add_node(interface_node);
    spdlog::debug("Running grpc interface");

    RCLCPP_INFO(rclcpp::get_logger("luci_interface"), "Running ROS2 executor...");

    executor.spin();

    // TODO: clp The grpc threads on the client dont exit properly because they are always in a
    // loop, I need to find a way to kill them on destructor and not just wait for a join. Will add
    // atomic bool flag to terminate
    rclcpp::shutdown();
    return 0;
}