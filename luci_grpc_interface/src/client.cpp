/**
 * @file client.cpp
 *
 * @copyright Copyright (c) 2022 Patroness, LLC. All Rights Reserved.
 *
 */

#include "../include/client.h"

using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientWriter;
using grpc::Status;
using Luci::ROS2::ClientGuide;
using sensors::DriveMode;
using sensors::ModeCtrl;
using sensors::RampMode;
using sensors::RemoteJsValues;
using sensors::Response;

pcl::PointCloud<pcl::PointXYZ> cameraPointCloud;
pcl::PointCloud<pcl::PointXYZ> ultrasonicPointCloud;
pcl::PointCloud<pcl::PointXYZ> radarPointCloud;

ClientGuide::ClientGuide(
    std::shared_ptr<Channel> channel, std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
    std::shared_ptr<DataBuffer<LuciZoneScaling>> zoneScalingDataBuff,
    std::shared_ptr<DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff,
    std::shared_ptr<DataBuffer<AhrsInfo>> ahrsInfoBuff)
    : stub_(sensors::Sensors::NewStub(channel)), joystickDataBuff(joystickDataBuff),
      cameraDataBuff(cameraDataBuff), radarDataBuff(radarDataBuff),
      ultrasonicDataBuff(ultrasonicDataBuff), zoneScalingDataBuff(zoneScalingDataBuff),
      joystickScalingDataBuff(joystickScalingDataBuff), ahrsInfoBuff(ahrsInfoBuff)
{
    grpcThreads.emplace_back(&ClientGuide::readJoystickPosition, this);
    grpcThreads.emplace_back(&ClientGuide::readCameraData, this);
    grpcThreads.emplace_back(&ClientGuide::readRadarData, this);
    grpcThreads.emplace_back(&ClientGuide::readUltrasonicData, this);
    grpcThreads.emplace_back(&ClientGuide::readZoneScalingData, this);
    grpcThreads.emplace_back(&ClientGuide::readJoystickScalingData, this);
    grpcThreads.emplace_back(&ClientGuide::readAhrsData, this);
}

ClientGuide::~ClientGuide()
{
    for (std::thread& grpcThread : this->grpcThreads)
    {
        grpcThread.join();
    }
    spdlog::debug("gRPC Ramp threads joined");
}

bool ClientGuide::activateEngagedMode() const
{
    ClientContext context;
    Response response;
    ModeCtrl request;
    request.set_mode(DriveMode::ENGAGED);

    if (Status status = stub_->SetDriveMode(&context, request, &response); status.ok())
    {
        spdlog::warn(response.reply());
    }
    else
    {
        return false;
    }
    return true;
}

bool ClientGuide::activateUserMode() const
{
    ClientContext context;
    Response response;
    ModeCtrl request;
    request.set_mode(DriveMode::USER);

    if (Status status = stub_->SetDriveMode(&context, request, &response); status.ok())
    {
        spdlog::warn(response.reply());
    }
    else
    {
        return false;
    }
    return true;
}

bool ClientGuide::activateAutoMode() const
{
    ClientContext context;
    Response response;
    ModeCtrl request;
    request.set_mode(DriveMode::AUTONOMOUS);

    if (Status status = stub_->SetDriveMode(&context, request, &response); status.ok())
    {
        spdlog::warn(response.reply());
    }
    else
    {
        return false;
    }
    return true;
}

int ClientGuide::sendJS(int forwardBack, int leftRight)
{
    // The forwardBack and the leftRight should be between 0-200 (100 being zero)
    ClientContext context;
    Response response;
    RemoteJsValues request;

    request.set_left_right(leftRight);
    request.set_forward_back(forwardBack);

    auto currentTime = std::chrono::high_resolution_clock::now();

    if (Status status = stub_->JsOverride(&context, request, &response); status.ok())
    {
        spdlog::info("Sending remote call... values ({} {}) status: {}",
                     std::to_string(forwardBack), std::to_string(leftRight), response.reply());
    }
    else
    {
        spdlog::error("Error communicating with server... pass in correct ip and port of chair");
        return 1;
    }
    spdlog::debug(
        "SEND JS: {}",
        std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - lastJSTime).count());
    lastJSTime = currentTime;
    return 0;
}

void ClientGuide::readJoystickPosition() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::Joystick response;

    std::unique_ptr<ClientReader<sensors::Joystick>> reader(
        stub_->JoystickStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        SystemJoystick joystickValues(response.forward_back(), response.left_right());

        this->joystickDataBuff->push(joystickValues);
    }
    spdlog::debug("joystick data buff closed");
    this->joystickDataBuff->close();
}

void ClientGuide::readUsbJoystickPosition() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::Joystick response;

    std::unique_ptr<ClientReader<sensors::Joystick>> reader(
        stub_->LuciJoystickStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        SystemJoystick joystickValues(response.forward_back(), response.left_right());

        this->joystickDataBuff->push(joystickValues);
    }
    spdlog::debug("joystick data buff closed");
    this->joystickDataBuff->close();
}

void ClientGuide::readCameraData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    CameraPoints response;

    std::unique_ptr<ClientReader<CameraPoints>> reader(stub_->CameraStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        cameraPointCloud.clear();

        for (auto point : response.points())
        {
            pcl::PointXYZ pclPoint;
            pclPoint.x = point.x();
            pclPoint.y = point.y();
            pclPoint.z = point.z();
            cameraPointCloud.points.push_back(pclPoint);
        }
        this->cameraDataBuff->push(cameraPointCloud);
    }
}

void ClientGuide::readUltrasonicData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    UltrasonicDistances response;

    std::unique_ptr<ClientReader<UltrasonicDistances>> reader(
        stub_->UltrasonicStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        ultrasonicPointCloud.clear();
        for (auto distance : response.distances())
        {
            for (auto point : distance.arc_points())
            {
                pcl::PointXYZ pclPoint;
                pclPoint.x = point.x();
                pclPoint.y = point.y();
                pclPoint.z = point.z();
                ultrasonicPointCloud.points.push_back(pclPoint);
            }
        }
        this->ultrasonicDataBuff->push(ultrasonicPointCloud);
    }
}
void ClientGuide::readRadarData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    RadarPoints response;

    std::unique_ptr<ClientReader<RadarPoints>> reader(stub_->RadarStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        radarPointCloud.clear();

        for (auto point : response.points())
        {
            pcl::PointXYZ pclPoint;
            pclPoint.x = point.x();
            pclPoint.y = point.y();
            pclPoint.z = point.z();
            radarPointCloud.points.push_back(pclPoint);
        }
        this->radarDataBuff->push(radarPointCloud);
    }
}

void ClientGuide::readZoneScalingData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    NavigationScaling response;

    std::unique_ptr<ClientReader<NavigationScaling>> reader(
        stub_->ScalingStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        LuciZoneScaling scalingValues(
            response.front_fb(), response.front_rl(), response.front_right_fb(),
            response.front_right_rl(), response.front_left_fb(), response.front_left_rl(),
            response.right_fb(), response.right_rl(), response.left_fb(), response.left_rl(),
            response.back_right_fb(), response.back_right_rl(), response.back_left_fb(),
            response.back_left_rl(), response.back_fb(), response.back_rl());

        this->zoneScalingDataBuff->push(scalingValues);
    }
}

void ClientGuide::readJoystickScalingData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    JoystickData response;

    std::unique_ptr<ClientReader<JoystickData>> reader(
        stub_->ScaledJoystickStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        LuciJoystickScaling scalingValues(response.forward_back(), response.left_right(),
                                          response.joystick_zone());

        this->joystickScalingDataBuff->push(scalingValues);
    }
}

void ClientGuide::readAhrsData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    AhrsData response;

    std::unique_ptr<ClientReader<AhrsData>> reader(stub_->AhrsStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        AhrsInfo ahrsInfo(response.linear_velocity().x(), response.linear_velocity().y(),
                          response.linear_velocity().z(), response.angular_velocity().x(),
                          response.angular_velocity().y(), response.angular_velocity().z());

        this->ahrsInfoBuff->push(ahrsInfo);
    }
}
