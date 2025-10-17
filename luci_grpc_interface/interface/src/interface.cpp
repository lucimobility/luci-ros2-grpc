/**
 * @file interface.cpp
 *
 * @brief Main interface node to connect ROS2 and gRPC
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

#include "grpc_interface/interface.h"

void Interface::processCameraData()
{
    // Wait on new camera data from gRPC and then send out converted ROS2 message
    while (true)
    {
        // Camera point cloud processing
        auto cameraPointCloud = this->cameraDataBuff->waitNext();

        RCLCPP_INFO(rclcpp::get_logger("luci_interface"), "Number of camera points: %ld",
                    cameraPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosCameraPointCloud;

        pcl::toROSMsg(cameraPointCloud, rosCameraPointCloud);
        std_msgs::msg::Header cameraHeader;
        cameraHeader.frame_id = "base_camera";
        cameraHeader.stamp = this->get_clock()->now();
        rosCameraPointCloud.header = cameraHeader;
        this->cameraPublisher->publish(rosCameraPointCloud);
    }
}

void Interface::processCollisionData()
{
    // Wait on new collision data from gRPC and then send out converted ROS2 message
    while (true)
    {
        auto collisionPointCloud = this->collisionDataBuff->waitNext();
        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Number of collision points: %ld",
                     collisionPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosCollisionPointCloud;
        pcl::toROSMsg(collisionPointCloud, rosCollisionPointCloud);
        std_msgs::msg::Header collisionHeader;
        collisionHeader.frame_id = "base_camera";
        collisionHeader.stamp = this->get_clock()->now();
        rosCollisionPointCloud.header = collisionHeader;
        this->collisionPointsPublisher->publish(rosCollisionPointCloud);
    }
}

void Interface::processDropoffData()
{
    // Wait on new dropoff data from gRPC and then send out converted ROS2 message
    while (true)
    {
        auto dropoffPointCloud = this->dropoffDataBuff->waitNext();
        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Number of dropoff points: %ld",
                     dropoffPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosDropoffPointCloud;
        pcl::toROSMsg(dropoffPointCloud, rosDropoffPointCloud);
        std_msgs::msg::Header dropoffHeader;
        dropoffHeader.frame_id = "base_camera";
        dropoffHeader.stamp = this->get_clock()->now();
        rosDropoffPointCloud.header = dropoffHeader;
        this->dropoffPointsPublisher->publish(rosDropoffPointCloud);
    }
}

void Interface::processRadarData()
{
    // Wait on new radar data from gRPC and then send out converted ROS2 message
    while (true)
    {
        auto radarPointCloud = this->radarDataBuff->waitNext();
        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Number of radar points: %ld",
                     radarPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosRadarPointCloud;
        pcl::toROSMsg(radarPointCloud, rosRadarPointCloud);
        std_msgs::msg::Header radarHeader;
        radarHeader.frame_id = "base_radar";
        radarHeader.stamp = this->get_clock()->now();
        rosRadarPointCloud.header = radarHeader;
        this->radarPublisher->publish(rosRadarPointCloud);
    }
}

void Interface::processUltrasonicData()
{
    // Wait on new ultrasonic data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto ultrasonicPointCloud = this->ultrasonicDataBuff->waitNext();
        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Number of ultrasonic points: %ld",
                     ultrasonicPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosUltrasonicPointCloud;
        pcl::toROSMsg(ultrasonicPointCloud, rosUltrasonicPointCloud);
        std_msgs::msg::Header ultrasonicHeader;
        ultrasonicHeader.frame_id = "base_ultrasonic";
        ultrasonicHeader.stamp = this->get_clock()->now();
        rosUltrasonicPointCloud.header = ultrasonicHeader;
        this->ultrasonicPublisher->publish(rosUltrasonicPointCloud);
    }
}
void Interface::processZoneScalingData()
{
    // Wait on new scaling data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto zoneScalingData = this->zoneScalingDataBuff->waitNext();
        luci_messages::msg::LuciZoneScaling zoneScalingMsg;
        zoneScalingMsg.front_fb = zoneScalingData.front_fb;
        zoneScalingMsg.front_rl = zoneScalingData.front_rl;
        zoneScalingMsg.front_right_fb = zoneScalingData.front_right_fb;
        zoneScalingMsg.front_right_rl = zoneScalingData.front_right_rl;
        zoneScalingMsg.front_left_fb = zoneScalingData.front_left_fb;
        zoneScalingMsg.front_left_rl = zoneScalingData.front_left_rl;
        zoneScalingMsg.right_fb = zoneScalingData.right_fb;
        zoneScalingMsg.right_rl = zoneScalingData.right_rl;
        zoneScalingMsg.left_fb = zoneScalingData.left_fb;
        zoneScalingMsg.left_rl = zoneScalingData.left_rl;
        zoneScalingMsg.back_right_fb = zoneScalingData.back_right_fb;
        zoneScalingMsg.back_right_rl = zoneScalingData.back_right_rl;
        zoneScalingMsg.back_left_fb = zoneScalingData.back_left_fb;
        zoneScalingMsg.back_left_rl = zoneScalingData.back_left_rl;
        zoneScalingMsg.back_fb = zoneScalingData.back_fb;
        zoneScalingMsg.back_rl = zoneScalingData.back_rl;

        this->zoneScalingPublisher->publish(zoneScalingMsg);
    }
}

void Interface::processJoystickScalingData()
{
    // Wait on new scaled joystick data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto joystickScalingData = this->joystickScalingDataBuff->waitNext();

        luci_messages::msg::LuciJoystickScaling joystickScalingMsg;
        joystickScalingMsg.forward_back = joystickScalingData.forward_back;
        joystickScalingMsg.left_right = joystickScalingData.left_right;
        joystickScalingMsg.joystick_zone = static_cast<int>(joystickScalingData.joystick_zone);
        joystickScalingMsg.input_source = static_cast<int>(joystickScalingData.input_source);
        joystickScalingMsg.forward_back_scaling = joystickScalingData.forward_back_scaling;
        joystickScalingMsg.left_right_scaling = joystickScalingData.left_right_scaling;
        this->joystickScalingPublisher->publish(joystickScalingMsg);
    }
}
void Interface::processJoystickPositionData()
{
    // Wait on new joystick data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto joystickPositionData = this->joystickDataBuff->waitNext();
        luci_messages::msg::LuciJoystick joystickPositionMsg;
        joystickPositionMsg.forward_back = joystickPositionData.forward_back;
        joystickPositionMsg.left_right = joystickPositionData.left_right;
        joystickPositionMsg.joystick_zone = static_cast<int>(joystickPositionData.joystick_zone);
        joystickPositionMsg.input_source = static_cast<int>(joystickPositionData.input_source);
        this->joystickPositionPublisher->publish(joystickPositionMsg);
    }
}
void Interface::processAhrsData()
{
    // Wait on new AHRS data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto ahrsData = this->ahrsInfoDataBuff->waitNext();

        nav_msgs::msg::Odometry rosOdomMsg;
        // Header
        std_msgs::msg::Header odomHeader;
        odomHeader.frame_id = "base_link";
        odomHeader.stamp = this->get_clock()->now();

        rosOdomMsg.header = odomHeader;
        // Child frame
        rosOdomMsg.child_frame_id = "base_link";
        // Twist with Covariance
        geometry_msgs::msg::TwistWithCovariance odomTwistCovariance;
        geometry_msgs::msg::Twist odomTwist;
        geometry_msgs::msg::Vector3 linearVelocity;
        linearVelocity.x = ahrsData.linear_velocity_x;
        linearVelocity.y = ahrsData.linear_velocity_y;
        linearVelocity.z = ahrsData.linear_velocity_z;
        odomTwist.linear = linearVelocity;
        geometry_msgs::msg::Vector3 angularVelocity;
        angularVelocity.x = ahrsData.angular_velocity_x;
        angularVelocity.y = ahrsData.angular_velocity_y;
        angularVelocity.z = ahrsData.angular_velocity_z;
        odomTwist.angular = angularVelocity;
        odomTwistCovariance.twist = odomTwist;
        rosOdomMsg.twist = odomTwistCovariance;

        this->odomPublisher->publish(rosOdomMsg);
    }
}

void Interface::processImuData()
{
    // Wait on new IMU data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto imuData = this->imuDataBuff->waitNext();

        luci_messages::msg::LuciImu rosImuMsg;
        // Header
        std_msgs::msg::Header imuHeader;
        imuHeader.frame_id = "base_link";
        imuHeader.stamp = this->get_clock()->now();

        rosImuMsg.header = imuHeader;
        rosImuMsg.quaternion_x = imuData.quaternion_x;
        rosImuMsg.quaternion_y = imuData.quaternion_y;
        rosImuMsg.quaternion_z = imuData.quaternion_z;
        rosImuMsg.quaternion_w = imuData.quaternion_w;

        rosImuMsg.acceleration_x = imuData.acceleration_x;
        rosImuMsg.acceleration_y = imuData.acceleration_y;
        rosImuMsg.acceleration_z = imuData.acceleration_z;

        rosImuMsg.gyro_x = imuData.gyro_x;
        rosImuMsg.gyro_y = imuData.gyro_y;
        rosImuMsg.gyro_z = imuData.gyro_z;

        rosImuMsg.euler_x = imuData.euler_x;
        rosImuMsg.euler_y = imuData.euler_y;
        rosImuMsg.euler_z = imuData.euler_z;

        rosImuMsg.accelerometer_x = imuData.accelerometer_x;
        rosImuMsg.accelerometer_y = imuData.accelerometer_y;
        rosImuMsg.accelerometer_z = imuData.accelerometer_z;

        rosImuMsg.magnetometer_x = imuData.magnetometer_x;
        rosImuMsg.magnetometer_y = imuData.magnetometer_y;
        rosImuMsg.magnetometer_z = imuData.magnetometer_z;

        rosImuMsg.gravity_x = imuData.gravity_x;
        rosImuMsg.gravity_y = imuData.gravity_y;
        rosImuMsg.gravity_z = imuData.gravity_z;

        rosImuMsg.cal_system = imuData.cal_system;
        rosImuMsg.cal_gyroscope = imuData.cal_gyroscope;
        rosImuMsg.cal_accelerometer = imuData.cal_accelerometer;
        rosImuMsg.cal_magnetometer = imuData.cal_magnetometer;
        rosImuMsg.source = imuData.source;

        this->imuPublisher->publish(rosImuMsg);
    }
}

void Interface::processEncoderData()
{
    // Wait on new Encoder data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto encoderData = this->encoderDataBuff->waitNext();

        luci_messages::msg::LuciEncoders rosEncoderMsg;
        // Header
        std_msgs::msg::Header encoderHeader;
        encoderHeader.frame_id = "base_link";
        encoderHeader.stamp = this->get_clock()->now();

        rosEncoderMsg.header = encoderHeader;

        rosEncoderMsg.left_angle = encoderData.left_angle;
        rosEncoderMsg.right_angle = encoderData.right_angle;

        rosEncoderMsg.fl_caster_degrees = encoderData.fl_caster_degrees;
        rosEncoderMsg.bl_caster_degrees = encoderData.bl_caster_degrees;
        rosEncoderMsg.fr_caster_degrees = encoderData.fr_caster_degrees;
        rosEncoderMsg.br_caster_degrees = encoderData.br_caster_degrees;

        rosEncoderMsg.edge_timestamp = encoderData.edge_timestamp;

        this->encoderPublisher->publish(rosEncoderMsg);
    }
}

void Interface::processLeftIrData()
{
    // Wait on new left ir data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto leftIrData = this->irDataBuffLeft->waitNext();

        sensor_msgs::msg::Image rosIrMsg;
        luci_messages::msg::LuciCameraInfo cameraInfoMsg;

        // Header
        std_msgs::msg::Header irHeader;
        irHeader.frame_id = "left_camera";
        irHeader.stamp = this->get_clock()->now();

        rosIrMsg.header = irHeader;
        cameraInfoMsg.header = irHeader;
        cameraInfoMsg.type = leftIrData.rotationType;

        rosIrMsg.height = leftIrData.height;
        rosIrMsg.width = leftIrData.width;

        rosIrMsg.data = leftIrData.data;

        rosIrMsg.encoding = "mono8";
        rosIrMsg.step = leftIrData.width * sizeof(uint8_t);

        cameraInfoMsg.intrinsics = {leftIrData.intrinsics.fx, leftIrData.intrinsics.fy,
                                    leftIrData.intrinsics.ppx, leftIrData.intrinsics.ppy};

        cameraInfoMsg.translation = {leftIrData.transform.translation[0],
                                     leftIrData.transform.translation[1],
                                     leftIrData.transform.translation[2]};
        cameraInfoMsg.rotation = {leftIrData.transform.rotation[0],
                                  leftIrData.transform.rotation[1],
                                  leftIrData.transform.rotation[2]};

        this->irLeftPublisher->publish(rosIrMsg);
        this->leftCameraInfoPublisher->publish(cameraInfoMsg);
    }
}

void Interface::processRightIrData()
{
    // Wait on new right ir data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto rightIrData = this->irDataBuffRight->waitNext();

        sensor_msgs::msg::Image rosIrMsg;
        luci_messages::msg::LuciCameraInfo cameraInfoMsg;

        // Header
        std_msgs::msg::Header irHeader;
        irHeader.frame_id = "right_camera";
        irHeader.stamp = this->get_clock()->now();

        rosIrMsg.header = irHeader;
        cameraInfoMsg.header = irHeader;
        cameraInfoMsg.type = rightIrData.rotationType;

        rosIrMsg.height = rightIrData.height;
        rosIrMsg.width = rightIrData.width;

        rosIrMsg.data = rightIrData.data;

        rosIrMsg.encoding = "mono8";
        rosIrMsg.step = rightIrData.width * sizeof(uint8_t);

        cameraInfoMsg.intrinsics = {rightIrData.intrinsics.fx, rightIrData.intrinsics.fy,
                                    rightIrData.intrinsics.ppx, rightIrData.intrinsics.ppy};

        cameraInfoMsg.translation = {rightIrData.transform.translation[0],
                                     rightIrData.transform.translation[1],
                                     rightIrData.transform.translation[2]};
        cameraInfoMsg.rotation = {rightIrData.transform.rotation[0],
                                  rightIrData.transform.rotation[1],
                                  rightIrData.transform.rotation[2]};

        this->irRightPublisher->publish(rosIrMsg);
        this->rightCameraInfoPublisher->publish(cameraInfoMsg);
    }
}

void Interface::processRearIrData()
{
    // Wait on new rear ir data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto rearIrData = this->irDataBuffRear->waitNext();

        sensor_msgs::msg::Image rosIrMsg;
        luci_messages::msg::LuciCameraInfo cameraInfoMsg;

        // Header
        std_msgs::msg::Header irHeader;
        irHeader.frame_id = "rear_camera";
        irHeader.stamp = this->get_clock()->now();

        rosIrMsg.header = irHeader;
        cameraInfoMsg.header = irHeader;
        cameraInfoMsg.type = rearIrData.rotationType;

        rosIrMsg.height = rearIrData.height;
        rosIrMsg.width = rearIrData.width;

        rosIrMsg.data = rearIrData.data;

        rosIrMsg.encoding = "mono8";
        rosIrMsg.step = rearIrData.width * sizeof(uint8_t);

        cameraInfoMsg.intrinsics = {rearIrData.intrinsics.fx, rearIrData.intrinsics.fy,
                                    rearIrData.intrinsics.ppx, rearIrData.intrinsics.ppy};

        cameraInfoMsg.translation = {rearIrData.transform.translation[0],
                                     rearIrData.transform.translation[1],
                                     rearIrData.transform.translation[2]};
        cameraInfoMsg.rotation = {rearIrData.transform.rotation[0],
                                  rearIrData.transform.rotation[1],
                                  rearIrData.transform.rotation[2]};

        this->irRearPublisher->publish(rosIrMsg);
        this->rearCameraInfoPublisher->publish(cameraInfoMsg);
    }
}

void Interface::processLeftDepthData()
{
    // Wait on new left depth data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto leftDepthData = this->depthDataBuffLeft->waitNext();

        sensor_msgs::msg::Image depthMsg;

        depthMsg.header.frame_id = "left_depth_camera";
        depthMsg.header.stamp = this->get_clock()->now();

        depthMsg.height = leftDepthData.height;
        depthMsg.width = leftDepthData.width;

        depthMsg.data = leftDepthData.data;

        depthMsg.encoding = "16UC1";
        depthMsg.step = leftDepthData.width * sizeof(uint16_t);

        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "depthMsg.data size %ld",
                     depthMsg.data.size());

        this->depthLeftPublisher->publish(depthMsg);
    }
}

void Interface::processRightDepthData()
{
    // Wait on new right depth data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto rightDepthData = this->depthDataBuffRight->waitNext();

        sensor_msgs::msg::Image depthMsg;

        depthMsg.header.frame_id = "right_depth_camera";
        depthMsg.header.stamp = this->get_clock()->now();

        depthMsg.height = rightDepthData.height;
        depthMsg.width = rightDepthData.width;

        depthMsg.data = rightDepthData.data;

        depthMsg.encoding = "16UC1";
        depthMsg.step = rightDepthData.width * sizeof(uint16_t);

        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "depthMsg.data size %ld",
                     depthMsg.data.size());

        this->depthRightPublisher->publish(depthMsg);
    }
}

void Interface::processRearDepthData()
{
    // Wait on new rear depth data from gRPC and then send out conveted ROS2 message
    while (true)
    {
        auto rearDepthData = this->depthDataBuffRear->waitNext();

        sensor_msgs::msg::Image depthMsg;

        depthMsg.header.frame_id = "rear_depth_camera";
        depthMsg.header.stamp = this->get_clock()->now();

        depthMsg.height = rearDepthData.height;
        depthMsg.width = rearDepthData.width;

        depthMsg.data = rearDepthData.data;

        depthMsg.encoding = "16UC1";
        depthMsg.step = rearDepthData.width * sizeof(uint16_t);

        RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "depthMsg.data size %ld",
                     depthMsg.data.size());

        this->depthRearPublisher->publish(depthMsg);
    }
}

void Interface::processChairProfileData()
{
    auto chairProfileData = this->chairProfileDataBuff->getLatest();
    std_msgs::msg::Int32 chairProfileMsg;
    if (chairProfileData.has_value())
    {
        chairProfileMsg.data = chairProfileData->profile;
        this->chairProfilePublisher->publish(chairProfileMsg);
    }
    while (true)
    {
        auto chairProfileData = this->chairProfileDataBuff->waitNext();
        std_msgs::msg::Int32 chairProfileMsg;
        chairProfileMsg.data = chairProfileData.profile;
        this->chairProfilePublisher->publish(chairProfileMsg);
    }
}

void Interface::processSpeedSettingData()
{
    auto speedSettingData = this->speedSettingDataBuff->getLatest();
    std_msgs::msg::Int32 speedSettingMsg;
    if (speedSettingData.has_value())
    {
        speedSettingMsg.data = speedSettingData->speed_setting;
        this->speedSettingPublisher->publish(speedSettingMsg);
    }

    while (true)
    {
        auto speedSettingData = this->speedSettingDataBuff->waitNext();
        std_msgs::msg::Int32 speedSettingMsg;
        speedSettingMsg.data = speedSettingData.speed_setting;
        this->speedSettingPublisher->publish(speedSettingMsg);
    }
}

void Interface::processOverrideButtonData()
{
    auto overrideButtonData = this->overrideButtonDataBuff->getLatest();
    std_msgs::msg::Int32 overrideButtonMsg;
    if (overrideButtonData.has_value())
    {
        overrideButtonMsg.data = overrideButtonData.value();
        this->overrideButtonDataPublisher->publish(overrideButtonMsg);
    }

    while (true)
    {
        auto overrideButtonData = this->overrideButtonDataBuff->waitNext();
        std_msgs::msg::Int32 overrideButtonMsg;
        overrideButtonMsg.data = overrideButtonData;
        this->overrideButtonDataPublisher->publish(overrideButtonMsg);
    }
}

void Interface::processOverrideButtonPressCountData()
{
    auto overrideButtonPressCountData = this->overrideButtonPressCountDataBuff->getLatest();
    std_msgs::msg::Int32 overrideButtonPressCountMsg;
    if (overrideButtonPressCountData.has_value())
    {
        overrideButtonPressCountMsg.data = overrideButtonPressCountData.value();
        this->overrideButtonPressCountDataPublisher->publish(overrideButtonPressCountMsg);
    }

    while (true)
    {
        auto overrideButtonPressCountData = this->overrideButtonPressCountDataBuff->waitNext();
        std_msgs::msg::Int32 overrideButtonPressCountMsg;
        overrideButtonPressCountMsg.data = overrideButtonPressCountData;
        this->overrideButtonPressCountDataPublisher->publish(overrideButtonPressCountMsg);
    }
}

void Interface::sendJsCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg)
{
    // Send the remote JS values over gRPC
    auto inputSource = static_cast<InputSource>(msg->input_source);
    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Received js val: %d %d %d",
                 msg->forward_back, msg->left_right, msg->input_source);

    this->luciInterface->sendJS(msg->forward_back + 100, msg->left_right + 100, inputSource);
}

void Interface::setSharedRemoteInputSource()
{
    this->luciInterface->setInputSource(InputSource::SharedRemote);
}

void Interface::removeSharedRemoteInputSource()
{
    this->luciInterface->removeInputSource(InputSource::SharedRemote);
}

void Interface::setAutoRemoteInputSource()
{
    this->luciInterface->setInputSource(InputSource::AutonomousRemote);
}

void Interface::removeAutoRemoteInputSource()
{
    this->luciInterface->removeInputSource(InputSource::AutonomousRemote);
}

void Interface::disableRadarFilter()
{
    // Disable the radar filter
    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Disabling radar filter");
    this->luciInterface->disableRadarFilter(RadarFilter::RANGE_CHOP);
    // this->luciInterface->disableRadarFilter(RadarFilter::ORIGIN);
    // this->luciInterface->disableRadarFilter(RadarFilter::FOV);
    // this->luciInterface->disableRadarFilter(RadarFilter::PEAK);
    this->luciInterface->disableRadarFilter(RadarFilter::STICKY);
    this->luciInterface->disableRadarFilter(RadarFilter::EXTRA_STICKY);
    // this->luciInterface->disableRadarFilter(RadarFilter::TRANSFORMS);
    this->luciInterface->disableRadarFilter(RadarFilter::ADAM);
}

void Interface::enableRadarFilter()
{
    // Enable the radar filter
    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Enabling radar filter");
    this->luciInterface->enableRadarFilter(RadarFilter::RANGE_CHOP);
    // this->luciInterface->enableRadarFilter(RadarFilter::ORIGIN);
    // this->luciInterface->enableRadarFilter(RadarFilter::FOV);
    // this->luciInterface->enableRadarFilter(RadarFilter::PEAK);
    this->luciInterface->enableRadarFilter(RadarFilter::STICKY);
    this->luciInterface->enableRadarFilter(RadarFilter::EXTRA_STICKY);
    // this->luciInterface->enableRadarFilter(RadarFilter::TRANSFORMS);
    this->luciInterface->enableRadarFilter(RadarFilter::ADAM);
}

void Interface::updateIrFrameRate(int rate)
{
    // Send the new rate request over gRPC to LUCI
    RCLCPP_DEBUG(rclcpp::get_logger("luci_interface"), "Received rate val: %d", rate);
    this->luciInterface->updateFrameRate(rate);
}