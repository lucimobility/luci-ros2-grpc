/**
 * @file client.cpp
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

using sensors::RemoteJsValues;
using sensors::Response;

using sensors::ActiveScaling;
using sensors::AhrsData;
using sensors::CameraMetaData;
using sensors::CameraPoints;
using sensors::CameraPoints2D;
using sensors::JoystickData;
using sensors::NavigationScaling;
using sensors::RadarPoints;
using sensors::UltrasonicDistances;
// using sensors::RadarFilter;

pcl::PointCloud<pcl::PointXYZ> cameraPointCloud;
pcl::PointCloud<pcl::PointXYZ> collisionPointCloud;
pcl::PointCloud<pcl::PointXYZ> dropoffPointCloud;
pcl::PointCloud<pcl::PointXYZ> ultrasonicPointCloud;
pcl::PointCloud<pcl::PointXYZ> radarPointCloud;

ClientGuide::ClientGuide(
    std::shared_ptr<Channel> channel, std::shared_ptr<DataBuffer<SystemJoystick>> joystickDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> collisionDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> dropoffDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff,
    std::shared_ptr<DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff,
    std::shared_ptr<DataBuffer<LuciZoneScaling>> zoneScalingDataBuff,
    std::shared_ptr<DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff,
    std::shared_ptr<DataBuffer<AhrsInfo>> ahrsDataBuff,
    std::shared_ptr<DataBuffer<ImuData>> imuDataBuff,
    std::shared_ptr<DataBuffer<EncoderData>> encoderDataBuff,
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffLeft,
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRight,
    std::shared_ptr<DataBuffer<CameraIrData>> irDataBuffRear, 
    std::shared_ptr<DataBuffer<CameraDepthData>> depthDataBuffLeft,
    std::shared_ptr<DataBuffer<CameraDepthData>> depthDataBuffRight,
    std::shared_ptr<DataBuffer<CameraDepthData>> depthDataBuffRear,
    int initialFrameRate, 
    std::shared_ptr<DataBuffer<ChairProfile>> chairProfileDataBuff,
    std::shared_ptr<DataBuffer<SpeedSetting>> speedSettingDataBuff,
    std::shared_ptr<DataBuffer<int>> overrideButtonDataBuff,
    std::shared_ptr<DataBuffer<int>> overrideButtonPressCountDataBuff)
    : stub_(sensors::Sensors::NewStub(channel)), joystickDataBuff(joystickDataBuff),
      cameraDataBuff(cameraDataBuff), collisionDataBuff(collisionDataBuff),
      dropoffDataBuff(dropoffDataBuff), radarDataBuff(radarDataBuff),
      ultrasonicDataBuff(ultrasonicDataBuff), zoneScalingDataBuff(zoneScalingDataBuff),
      joystickScalingDataBuff(joystickScalingDataBuff), ahrsDataBuff(ahrsDataBuff),
      imuDataBuff(imuDataBuff), encoderDataBuff(encoderDataBuff), irDataBuffLeft(irDataBuffLeft),
      irDataBuffRight(irDataBuffRight), irDataBuffRear(irDataBuffRear), chairProfileDataBuff(chairProfileDataBuff),
      speedSettingDataBuff(speedSettingDataBuff), depthDataBuffLeft(depthDataBuffLeft),
      depthDataBuffRight(depthDataBuffRight), depthDataBuffRear(depthDataBuffRear), overrideButtonDataBuff(overrideButtonDataBuff),
      overrideButtonPressCountDataBuff(overrideButtonPressCountDataBuff)
{
    grpcThreads.emplace_back(&ClientGuide::readJoystickPosition, this);
    grpcThreads.emplace_back(&ClientGuide::readCameraData, this);
    grpcThreads.emplace_back(&ClientGuide::readCollisionData, this);
    grpcThreads.emplace_back(&ClientGuide::readDropoffData, this);
    grpcThreads.emplace_back(&ClientGuide::readRadarData, this);
    grpcThreads.emplace_back(&ClientGuide::readUltrasonicData, this);
    grpcThreads.emplace_back(&ClientGuide::readZoneScalingData, this);
    grpcThreads.emplace_back(&ClientGuide::readJoystickScalingData, this);
    grpcThreads.emplace_back(&ClientGuide::readAhrsData, this);
    grpcThreads.emplace_back(&ClientGuide::readImuData, this);
    grpcThreads.emplace_back(&ClientGuide::readEncoderData, this);
    grpcThreads.emplace_back(&ClientGuide::readIrFrame, this, initialFrameRate);
    grpcThreads.emplace_back(&ClientGuide::readDepthFrame, this);
    grpcThreads.emplace_back(&ClientGuide::readChairProfile, this);
    grpcThreads.emplace_back(&ClientGuide::readSpeedSetting, this);
    grpcThreads.emplace_back(&ClientGuide::readOverrideButtonData, this);
    grpcThreads.emplace_back(&ClientGuide::readOverrideButtonPressCountData, this);
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
    default:
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Unexpected luci joystick zone %d, defaulting to origin",
            static_cast<int>(zone));
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
    default:
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Unexpected sensors joystick zone %d, defaulting to origin",
            static_cast<int>(zone));
    }
    return JoystickZone::Origin;
}

sensors::InputSource convertInputSourceToProto(const InputSource inputSource)
{
    switch (inputSource)
    {
    case InputSource::RampAssist:
        return sensors::InputSource::RampAssist;
    case InputSource::AutonomousRemote:
        return sensors::InputSource::AutonomousRemote;
    case InputSource::WDI:
        return sensors::InputSource::WDI;
    case InputSource::ChairVirtual:
        return sensors::InputSource::ChairVirtual;
    case InputSource::ChairPhysical:
        return sensors::InputSource::ChairPhysical;
    case InputSource::SharedRemote:
        return sensors::InputSource::SharedRemote;
    default:
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Unexpected luci input source %d, defaulting to chair virtual input source",
            static_cast<int>(inputSource));
    }
    return sensors::InputSource::ChairVirtual;
}

InputSource convertProtoInputSource(const sensors::InputSource inputSource)
{
    switch (inputSource)
    {
    case sensors::InputSource::RampAssist:
        return InputSource::RampAssist;
    case sensors::InputSource::AutonomousRemote:
        return InputSource::AutonomousRemote;
    case sensors::InputSource::WDI:
        return InputSource::WDI;
    case sensors::InputSource::ChairVirtual:
        return InputSource::ChairVirtual;
    case sensors::InputSource::ChairPhysical:
        return InputSource::ChairPhysical;
    case sensors::InputSource::SharedRemote:
        return InputSource::SharedRemote;
    default:
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Unexpected sensors input source %d, defaulting to chair virtual input source",
            static_cast<int>(inputSource));
    }
    return InputSource::ChairVirtual;
}

RadarFilter convertProtoRadarFilter(const sensors::RadarFilter_Filter filter)
{
    switch (filter)
    {
    case sensors::RadarFilter_Filter::RadarFilter_Filter_RANGE_CHOP:
        return RadarFilter::RANGE_CHOP;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_ORIGIN:
        return RadarFilter::ORIGIN;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_FOV:
        return RadarFilter::FOV;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_PEAK:
        return RadarFilter::PEAK;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_STICKY:
        return RadarFilter::STICKY;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_EXTRA_STICKY:
        return RadarFilter::EXTRA_STICKY;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_TRANSFORMS:
        return RadarFilter::TRANSFORMS;
    case sensors::RadarFilter_Filter::RadarFilter_Filter_ADAM:
        return RadarFilter::ADAM;
    }
}

sensors::RadarFilter_Filter convertRadarFilterToProto(const RadarFilter filter)
{
    switch (filter)
    {
    case RadarFilter::RANGE_CHOP:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_RANGE_CHOP;
    case RadarFilter::ORIGIN:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_ORIGIN;
    case RadarFilter::FOV:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_FOV;
    case RadarFilter::PEAK:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_PEAK;
    case RadarFilter::STICKY:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_STICKY;
    case RadarFilter::EXTRA_STICKY:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_EXTRA_STICKY;
    case RadarFilter::TRANSFORMS:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_TRANSFORMS;
    case RadarFilter::ADAM:
        return sensors::RadarFilter_Filter::RadarFilter_Filter_ADAM;
    }   
}

ClientGuide::~ClientGuide()
{
    for (std::thread& grpcThread : this->grpcThreads)
    {
        grpcThread.join();
    }
    RCLCPP_INFO(
        rclcpp::get_logger("luci_interface"), "gRPC Ramp threads joined");
}

int ClientGuide::setInputSource(InputSource source) const
{
    ClientContext context;
    sensors::InputSourceRequest request;
    Response response;

    request.set_source(convertInputSourceToProto(source));

    if (Status status = stub_->AddInputSource(&context, request, &response); status.ok())
    {
        RCLCPP_INFO(
            rclcpp::get_logger("luci_interface"),
            "Setting input source to %d", static_cast<int>(source));
        return 0;
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Error communicating with server... pass in correct ip and port of chair");
        return 1;
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
        RCLCPP_INFO(
            rclcpp::get_logger("luci_interface"),
            "Removing input source %d", static_cast<int>(source));
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Error communicating with server... pass in correct ip and port of chair");
    }
}

void ClientGuide::disableRadarFilter(RadarFilter filter) const
{
    ClientContext context;
    sensors::RadarFilter request;
    Response response;

    request.set_filter(convertRadarFilterToProto(filter));

    if (Status status = stub_->DisableRadarFilter(&context, request, &response); status.ok())
    {
        RCLCPP_INFO(
            rclcpp::get_logger("luci_interface"),
            "Disabling radar filter %d", static_cast<int>(filter));
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Error communicating with server... pass in correct ip and port of chair");
    }
}

void ClientGuide::enableRadarFilter(RadarFilter filter) const
{
    ClientContext context;
    sensors::RadarFilter request;
    Response response;

    request.set_filter(convertRadarFilterToProto(filter));

    if (Status status = stub_->EnableRadarFilter(&context, request, &response); status.ok())
    {
        RCLCPP_INFO(
            rclcpp::get_logger("luci_interface"),
            "Enabling radar filter %d", static_cast<int>(filter));
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Error communicating with server... pass in correct ip and port of chair");
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
        RCLCPP_INFO(
            rclcpp::get_logger("luci_interface"),
            "Sending remote call... values (%d %d %d) status: %s",
            forwardBack, leftRight, static_cast<int>(source), response.reply().c_str());
    }
    else
    {
        RCLCPP_ERROR(
            rclcpp::get_logger("luci_interface"),
            "Error communicating with server... pass in correct ip and port of chair");
        return 1;
    }
    RCLCPP_DEBUG(
        rclcpp::get_logger("luci_interface"),
        "SEND JS: %ld",
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
    RCLCPP_INFO(
        rclcpp::get_logger("luci_interface"), "joystick data buff closed");
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

void ClientGuide::readCollisionData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    CameraPoints2D response;

    std::unique_ptr<ClientReader<CameraPoints2D>> reader(
        stub_->FlatCameraStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        collisionPointCloud.clear();

        for (auto point : response.points())
        {
            pcl::PointXYZ pclPoint;
            pclPoint.x = point.x();
            pclPoint.y = point.y();
            pclPoint.z = 0.0;
            collisionPointCloud.push_back(pclPoint);
        }
        this->collisionDataBuff->push(collisionPointCloud);
    }
}

void ClientGuide::readDropoffData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    CameraPoints response;

    std::unique_ptr<ClientReader<CameraPoints>> reader(
        stub_->DropoffCameraStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        dropoffPointCloud.clear();

        for (auto point : response.points())
        {
            pcl::PointXYZ pclPoint;
            pclPoint.x = point.x();
            pclPoint.y = point.y();
            pclPoint.z = point.z();
            dropoffPointCloud.push_back(pclPoint);
        }
        this->dropoffDataBuff->push(dropoffPointCloud);
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
    ActiveScaling response;

    std::unique_ptr<ClientReader<ActiveScaling>> reader(
        stub_->ActiveScalingStream(&context, request));
    reader->Read(&response);

    while (reader->Read(&response))
    {
        LuciJoystickScaling scalingValues(
            response.joystick().forward_back(), response.joystick().left_right(),
            convertProtoZone(response.joystick().joystick_zone()),
            convertProtoInputSource(response.joystick().source()), response.forward_back_scaling(),
            response.left_right_scaling());

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
        RCLCPP_ERROR(rclcpp::get_logger("luci_interface"), "Failed to set initial rate");
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
                    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "IR Rate Sending: %d",
                                  requestedRate.rate());

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
            RCLCPP_ERROR(rclcpp::get_logger("luci_interface"),
                          "IR rotation type not valid defaulting to radians");
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

        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "IR MESSAGE SIZE: %ld",
                      response.ByteSizeLong());
    }

    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "IR data buff closed");
    this->irDataBuffLeft->close();
    this->irDataBuffRight->close();
    this->irDataBuffRear->close();

    // When the stream goes down sets the atomic to terminate this thread
    shutdown = true;
    rateReaderPublisher.join();
}

void ClientGuide::readDepthFrame()
{
    ClientContext context;
    sensors::DepthFrame response;

    std::unique_ptr<ClientReader<sensors::DepthFrame>> reader(
        stub_->DepthImageStream(&context, google::protobuf::Empty()));
    reader->Read(&response);

    while (reader->Read(&response)){
        CameraDepthData cameraDepthData (
            response.width(), response.height(),
            std::vector<uint8_t>(response.frame().begin(), response.frame().end()));
        if (response.camera() == "left")
        {
            this->depthDataBuffLeft->push(cameraDepthData);
        }
        else if (response.camera() == "right")
        {
            this->depthDataBuffRight->push(cameraDepthData);
        }
        else if (response.camera() == "rear")
        {
            this->depthDataBuffRear->push(cameraDepthData);
        }
        
        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Depth MESSAGE SIZE: %ld",
                      response.ByteSizeLong());
    } 
    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Depth data buff closed");
    this->depthDataBuffLeft->close();
    this->depthDataBuffRight->close();
    this->depthDataBuffRear->close();
    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Depth data buff closed");
}

void ClientGuide::readChairProfile() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::ChairProfile response;

    std::unique_ptr<ClientReader<sensors::ChairProfile>> reader(
        stub_->ChairProfileStream(&context, request));

    while (reader->Read(&response))
    {
        ChairProfile chairProfile(response.profile());
        RCLCPP_DEBUG(
            rclcpp::get_logger("luci_interface"), "Chair profile: %d", chairProfile.profile);
        this->chairProfileDataBuff->push(chairProfile);
    }
}

void ClientGuide::readSpeedSetting() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::SpeedSetting response;

    std::unique_ptr<ClientReader<sensors::SpeedSetting>> reader(
        stub_->SpeedSettingStream(&context, request));

    while (reader->Read(&response))
    {
        SpeedSetting speedSetting(response.speed_setting());
        RCLCPP_DEBUG(
            rclcpp::get_logger("luci_interface"), "Speed setting: %d", speedSetting.speed_setting);
        this->speedSettingDataBuff->push(speedSetting);
    }
}

void ClientGuide::readOverrideButtonData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::HmiStreamData response;

    std::unique_ptr<ClientReader<sensors::HmiStreamData>> reader(
        stub_->HmiStream(&context, request));

    while (reader->Read(&response))
    {
        int overrideButtonData(response.button_state());
        RCLCPP_DEBUG(
            rclcpp::get_logger("luci_interface"), "Override button data: %d", overrideButtonData);
        this->overrideButtonDataBuff->push(overrideButtonData);
    }
}

void ClientGuide::readOverrideButtonPressCountData() const
{
    ClientContext context;
    const google::protobuf::Empty request;
    sensors::PressCountStreamData response;

    std::unique_ptr<ClientReader<sensors::PressCountStreamData>> reader(
        stub_->PressCountStream(&context, request));

    while (reader->Read(&response))
    {
        int overrideButtonPressCount(response.press_count());
        RCLCPP_DEBUG(
            rclcpp::get_logger("luci_interface"), "Override button press count: %d", overrideButtonPressCount);
        this->overrideButtonPressCountDataBuff->push(overrideButtonPressCount);
    }
}
