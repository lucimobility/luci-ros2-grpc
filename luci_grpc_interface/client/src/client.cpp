/**
 * @file client.cpp
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
 *
 */

#include "client/client.h"

using namespace std::chrono_literals;
using grpc::Channel;
using grpc::ClientContext;
using grpc::ClientReader;
using grpc::ClientReaderWriter;
using grpc::ClientWriter;
using grpc::Status;

using Luci::ROS2::ClientGuide;

using sensors::DriveMode;
using sensors::ModeCtrl;
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
    std::shared_ptr<DataBuffer<SystemJoystick>> joystickScalingDataBuff,
    std::shared_ptr<DataBuffer<AhrsInfo>> ahrsDataBuff,
    std::shared_ptr<DataBuffer<ImuData>> imuDataBuff,
    std::shared_ptr<DataBuffer<EncoderData>> encoderDataBuff,
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffLeft,
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRight,
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRear, int initialFrameRate)
    : stub_(sensors::Sensors::NewStub(channel)), joystickDataBuff(joystickDataBuff),
      cameraDataBuff(cameraDataBuff), radarDataBuff(radarDataBuff),
      ultrasonicDataBuff(ultrasonicDataBuff), zoneScalingDataBuff(zoneScalingDataBuff),
      joystickScalingDataBuff(joystickScalingDataBuff), ahrsDataBuff(ahrsDataBuff),
      imuDataBuff(imuDataBuff), encoderDataBuff(encoderDataBuff), irDataBuffLeft(irDataBuffLeft),
      irDataBuffRight(irDataBuffRight), irDataBuffRear(irDataBuffRear)
{
    grpcThreads.emplace_back(&ClientGuide::readJoystickPosition, this);
    grpcThreads.emplace_back(&ClientGuide::readCameraData, this);
    grpcThreads.emplace_back(&ClientGuide::readRadarData, this);
    grpcThreads.emplace_back(&ClientGuide::readUltrasonicData, this);
    grpcThreads.emplace_back(&ClientGuide::readZoneScalingData, this);
    grpcThreads.emplace_back(&ClientGuide::readJoystickScalingData, this);
    grpcThreads.emplace_back(&ClientGuide::readAhrsData, this);
    grpcThreads.emplace_back(&ClientGuide::readImuData, this);
    grpcThreads.emplace_back(&ClientGuide::readEncoderData, this);
    grpcThreads.emplace_back(&ClientGuide::readIrFrame, this, initialFrameRate);
}

sensors::JoystickZone convertJoystickZoneToProto(const JoystickZone zone)
{
    switch (zone)
    {
    case JoystickZone::Front:
        return sensors::JoystickZone::Front;
    case JoystickZone::FrontLeft:
        return sensors::JoystickZone::FrontLeft;
    case JoystickZone::FrontRight:
        return sensors::JoystickZone::FrontRight;
    case JoystickZone::Left:
        return sensors::JoystickZone::Left;
    case JoystickZone::Right:
        return sensors::JoystickZone::Right;
    case JoystickZone::BackLeft:
        return sensors::JoystickZone::BackLeft;
    case JoystickZone::BackRight:
        return sensors::JoystickZone::BackRight;
    case JoystickZone::Back:
        return sensors::JoystickZone::Back;
    case JoystickZone::Origin:
        return sensors::JoystickZone::Origin;
        // default:
        // spdlog::error("Unexpected luci joystick zone {}, defaulting to origin", zone);
    }
    return sensors::JoystickZone::Origin;
}

JoystickZone convertProtoZone(const sensors::JoystickZone zone)
{
    switch (zone)
    {
    case sensors::JoystickZone::Front:
        return JoystickZone::Front;
    case sensors::JoystickZone::FrontLeft:
        return JoystickZone::FrontLeft;
    case sensors::JoystickZone::FrontRight:
        return JoystickZone::FrontRight;
    case sensors::JoystickZone::Left:
        return JoystickZone::Left;
    case sensors::JoystickZone::Right:
        return JoystickZone::Right;
    case sensors::JoystickZone::BackLeft:
        return JoystickZone::BackLeft;
    case sensors::JoystickZone::BackRight:
        return JoystickZone::BackRight;
    case sensors::JoystickZone::Back:
        return JoystickZone::Back;
    case sensors::JoystickZone::Origin:
        return JoystickZone::Origin;
        // default:
        // spdlog::error("Unexpected sensors joystick zone {}, defaulting to origin", zone);
    }
    return JoystickZone::Origin;
}

sensors::InputSource convertInputSourceToProto(const InputSource inputSource)
{
    switch (inputSource)
    {
    case InputSource::RampAssist:
        return sensors::InputSource::RampAssist;
    case InputSource::Remote:
        return sensors::InputSource::Remote;
    case InputSource::WDI:
        return sensors::InputSource::WDI;
    case InputSource::ChairVirtual:
        return sensors::InputSource::ChairVirtual;
    case InputSource::ChairPhysical:
        return sensors::InputSource::ChairPhysical;
        // default:
        //     spdlog::error("Unexpected luci input source {}, defaulting to chair virtual input
        //     source",
        //                   inputSource);
    }
    return sensors::InputSource::ChairVirtual;
}

InputSource convertProtoInputSource(const sensors::InputSource inputSource)
{
    switch (inputSource)
    {
    case sensors::InputSource::RampAssist:
        return InputSource::RampAssist;
    case sensors::InputSource::Remote:
        return InputSource::Remote;
    case sensors::InputSource::WDI:
        return InputSource::WDI;
    case sensors::InputSource::ChairVirtual:
        return InputSource::ChairVirtual;
    case sensors::InputSource::ChairPhysical:
        return InputSource::ChairPhysical;
        // default:
        //     spdlog::error(
        //         "Unexpected sensors input source {}, defaulting to chair virtual input source",
        //         inputSource);
    }
    return InputSource::ChairVirtual;
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

void ClientGuide::setInputSource(InputSource source) const
{
    ClientContext context;
    sensors::InputSourceRequest request;
    Response response;

    request.set_source(convertInputSourceToProto(source));

    if (Status status = stub_->AddInputSource(&context, request, &response); status.ok())
    {
        spdlog::info("Setting input source to {}", static_cast<int>(source));
    }
    else
    {
        spdlog::error("Error communicating with server... pass in correct ip and port of chair");
    }
}

void ClientGuide::removeInputSource(InputSource source) const
{
    ClientContext context;
    sensors::InputSourceRequest request;
    Response response;

    request.set_source(convertInputSourceToProto(source));

    if (Status status = stub_->RemoveInputSource(&context, request, &response); status.ok())
    {
        spdlog::info("Removing input source {}", static_cast<int>(source));
    }
    else
    {
        spdlog::error("Error communicating with server... pass in correct ip and port of chair");
    }
}

int ClientGuide::sendJS(int forwardBack, int leftRight, InputSource source)
{
    // The forwardBack and the leftRight should be between 0-200 (100 being zero)
    ClientContext context;
    Response response;
    RemoteJsValues request;

    request.set_left_right(leftRight);
    request.set_forward_back(forwardBack);
    request.set_source(convertInputSourceToProto(source));

    auto currentTime = std::chrono::high_resolution_clock::now();

    if (Status status = stub_->JsOverride(&context, request, &response); status.ok())
    {
        spdlog::info("Sending remote call... values ({} {} {}) status: {}",
                     std::to_string(forwardBack), std::to_string(leftRight),
                     std::to_string(static_cast<int>(source)), response.reply());
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
    sensors::JoystickData response;

    std::unique_ptr<ClientReader<sensors::JoystickData>> reader(
        stub_->UserJoystickStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        SystemJoystick joystickValues(response.forward_back(), response.left_right(),
                                      convertProtoZone(response.joystick_zone()),
                                      convertProtoInputSource(response.source()));

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

    std::unique_ptr<ClientReader<CameraPoints>> reader(
        stub_->DecimatedCameraStream(&context, request));
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
            cameraPointCloud.push_back(pclPoint);
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
        stub_->DriveJoystickStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        SystemJoystick scalingValues(response.forward_back(), response.left_right(),
                                     convertProtoZone(response.joystick_zone()),
                                     convertProtoInputSource(response.source()));

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

        this->ahrsDataBuff->push(ahrsInfo);
    }
}

void ClientGuide::readImuData() const
{
    ClientContext context;
    sensors::ImuCtrl request;
    request.set_imu(sensors::Imu::MPU);
    sensors::ImuData response;

    std::unique_ptr<ClientReader<sensors::ImuData>> reader(stub_->ImuStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        if (response.source() == sensors::Imu::MPU)
        {
            ImuData imuData(
                response.quaternion_x(), response.quaternion_y(), response.quaternion_z(),
                response.quaternion_w(), response.acceleration_x(), response.acceleration_y(),
                response.acceleration_z(), response.gyro_x(), response.gyro_y(), response.gyro_z(),
                response.euler_x(), response.euler_y(), response.euler_z(),
                response.accelerometer_x(), response.accelerometer_y(), response.accelerometer_z(),
                response.magnetometer_x(), response.magnetometer_y(), response.magnetometer_z(),
                response.gravity_x(), response.gravity_y(), response.gravity_z(),
                response.cal_system(), response.cal_gyroscope(), response.cal_accelerometer(),
                response.cal_magnetometer(), 2);

            this->imuDataBuff->push(imuData);
        }
    }
}

void ClientGuide::readEncoderData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::EncoderData response;

    std::unique_ptr<ClientReader<sensors::EncoderData>> reader(
        stub_->EncoderStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        EncoderData encoderData(response.left_angle(), response.right_angle(),
                                response.fl_caster_degrees(), response.bl_caster_degrees(),
                                response.fr_caster_degrees(), response.br_caster_degrees(),
                                response.edge_timestamp());

        this->encoderDataBuff->push(encoderData);
    }
}

void ClientGuide::updateFrameRate(int rate) { this->irFrameRateDataBuff.push(rate); }

void ClientGuide::readIrFrame(int initialRate)
{
    ClientContext context;
    sensors::IrFrame response;

    // Set up a bidirectional stream
    std::unique_ptr<ClientReaderWriter<sensors::FrameRate, sensors::IrFrame>> stream(
        stub_->IrStream(&context));

    // Send the inital requested rate
    sensors::FrameRate requestedRate;
    requestedRate.set_rate(initialRate);
    if (!stream->Write(requestedRate))
    {
        spdlog::error("Failed to set initial rate");
        return;
    }

    // Flag to signal the rate publisher to close down
    std::atomic_bool shutdown = false;
    // Thread for updating frame rate during execution of stream
    std::thread rateReaderPublisher(
        [&stream, &shutdown, this]()
        {
            sensors::FrameRate requestedRate;

            // Spin forever waiting for a new message on the rate data buff indicating a new
            // requested rate unless shutdown flag is set to true
            std::chrono::milliseconds threadWaitTime = 100ms;
            while (!shutdown)
            {
                // Wait for new data or threadWaitTime (ms) to pass. The timeout here is for
                // terminating the thread if the stream is closed.
                auto updateRateOpt = this->irFrameRateDataBuff.waitNext(threadWaitTime);
                if (updateRateOpt.has_value())
                {
                    auto updateRate = *updateRateOpt;
                    requestedRate.set_rate(updateRate);

                    // Based on how the writer below works the new rate will only take into effect
                    // once a frame has been set from the old rate. So at most the frame rate will
                    // need to wait 1 second before changing
                    spdlog::debug("IR Rate Sending: {}", requestedRate.rate());

                    if (!stream->Write(requestedRate))
                    {
                        break;
                    }
                }
            }
        });

    // Reading IR frame
    while (stream->Read(&response) && (response.frame().size() > 0))
    {
        // Make a new pointer to an array of bytes of image size
        auto& frame = response.frame();

        std::vector<uint8_t> bytes(frame.begin(), frame.end());
        sensors::CameraMetaData metaData = response.meta_data();

        CameraIntrinsics intrinsics =
            CameraIntrinsics(metaData.intrinsics().fx(), metaData.intrinsics().fy(),
                             metaData.intrinsics().ppx(), metaData.intrinsics().ppy());
        CameraTransform transform = CameraTransform();

        // Translation in meters
        transform.translation.push_back(metaData.transformation().translation().x());
        transform.translation.push_back(metaData.transformation().translation().y());
        transform.translation.push_back(metaData.transformation().translation().z());

        // Rotation in Radians
        transform.rotation.push_back(metaData.transformation().rotation().x());
        transform.rotation.push_back(metaData.transformation().rotation().y());
        transform.rotation.push_back(metaData.transformation().rotation().z());

        int type = 0;
        switch (metaData.transformation().type())
        {
        case sensors::Pose::RADIAN:
            type = 0;
            break;
        case sensors::Pose::DEGREE:
            type = 1;
            break;
        case sensors::Pose::QUATERNION:
            type = 2;
            break;

        default:
            spdlog::error("IR rotation type not valid defaulting to radians");
            break;
        }

        CameraIrData cameraIrData =
            CameraIrData(response.width(), response.height(), intrinsics, transform, type, bytes);
        {
            if (response.camera() == "left")
            {
                this->irDataBuffLeft->push(cameraIrData);
            }
            else if (response.camera() == "right")
            {
                this->irDataBuffRight->push(cameraIrData);
            }
            else if (response.camera() == "rear")
            {
                this->irDataBuffRear->push(cameraIrData);
            }
        }

        spdlog::debug("IR MESSAGE SIZE: {}", response.ByteSizeLong());
    }
    spdlog::debug("IR data buff closed");
    this->irDataBuffLeft->close();
    this->irDataBuffRight->close();
    this->irDataBuffRear->close();

    // When the stream goes down sets the atomic to terminate this thread
    shutdown = true;
    rateReaderPublisher.join();
}
