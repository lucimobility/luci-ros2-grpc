#include "../include/interface.h"
#include "../include/differentiator.h"
#include <luci_messages/msg/luci_joystick.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

std::vector<std::thread> processThreads;

// #include <spdlog/spdlog.h>

// ClientGuide* luciInterface = new ClientGuide(grpc::CreateChannel(host + ":" + port,
// grpc::InsecureChannelCredentials())); std::cout << "Client created at " << host << port <<
// std::endl;

// void Interface::sendJSCallback(const translator::luci_joystickConstPtr& msg)
void Interface::sendJSCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg)
{
    std::cout << "Recieved js val: " << msg->forward_back << " " << msg->left_right << std::endl;

    this->luciInterface->sendJS(msg->forward_back + 100, msg->left_right + 100);
    // RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
}

// Interface::Interface() : spinner(4)
// {
//     grpcThreads.emplace_back(&ClientGuide::readAhrsData, luciInterface);
//     grpcThreads.emplace_back(&ClientGuide::readCameraPointData, luciInterface);
//     // grpcThreads.emplace_back(&ClientGuide::readRadarPointData, luciInterface);
//     // grpcThreads.emplace_back(&ClientGuide::readUltrasonicPointData, luciInterface);

//     grpcThreads.emplace_back(&ClientGuide::readEncoderData, luciInterface);

//     spinner.start();
// }

float degreesTraveled(float current, float previous, bool leftWheel)
{
    float degreeDelta = 0.0;
    float currentPreviousDiff = current - previous;
    float previousCurrentDiff = previous - current;
    if (current > previous)
    {
        if (currentPreviousDiff > 250.0)
        {
            degreeDelta = -1 * ((360.0 - current) + previous);
        }
        else
        {
            degreeDelta = currentPreviousDiff;
        }
    }
    else if (current < previous)
    {
        if (previousCurrentDiff > 250.0)
        {
            degreeDelta = (360.0 - previous) + current;
        }
        else
        {
            degreeDelta = -1 * previousCurrentDiff;
        }
    }
    else if (current == previous)
    {
        return degreeDelta;
    }

    if (!leftWheel)
    {
        degreeDelta = -1 * degreeDelta;
    }
    return degreeDelta;
}

auto createQuaternionMsgFromYaw(double yaw)
{
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw);
    return tf2::toMsg(q);
}

int main(int argc, char** argv)
{

    std::cout << "THIS ONE" << std::endl;

    rclcpp::init(argc, argv);

    // TODO: figure out spin rate stuff
    auto interfaceNode = std::make_shared<Interface>();
    RCLCPP_ERROR(interfaceNode->get_logger(), "Started...");
    // std::string host = "10.1.10.182";
    // std::string port = "50051";
    std::vector<std::thread> grpcThreads;

    // interfaceNode->luciInterface =
    //     new ClientGuide(grpc::CreateChannel(host + ":" + port,
    //     grpc::InsecureChannelCredentials()));

    grpcThreads.emplace_back(&ClientGuide::readAhrsData, interfaceNode->luciInterface);
    grpcThreads.emplace_back(&ClientGuide::readCameraPointData, interfaceNode->luciInterface);
    // grpcThreads.emplace_back(&ClientGuide::readEncoderData, luciInterface);
    // rclcpp::spin(interfaceNode);

    rclcpp::Rate loop_rate(20);

    // Interface* interface = new Interface();
    // ros::Rate rate(20);

    // CHECK THE POINTCLOUD TO MAKE SURE ITS RIGHT WAY UP
    // if (interfaceNode->luciInterface->activateAutoMode())
    // {
    //     std::cout << "it made it" << std::endl;
    // };
    // interfaceNode->luciInterface->activateUserMode();

    // processThreads.emplace_back(&Interface::run, interface);

    // Encoder stuff
    Differentiator leftDiff;
    Differentiator rightDiff;
    bool first = true;
    float lastLeft = 0.0;
    float lastRight = 0.0;

    float totalDegreesLeft = 0.0;
    float totalDegreesRight = 0.0;

    // interface->currentTime = ros::Time::now();
    // interface->lastTime = ros::Time::now();

    if (interfaceNode->luciInterface->activateAutoMode())
    {
        std::cout << "it made it" << std::endl;
    };

    while (rclcpp::ok())
    {
        // RCLCPP_ERROR(interfaceNode->get_logger(), "Running...");
        // interface->currentTime = ros::Time::now();

        // auto encoderData = interfaceNode->luciInterface->getEncoderData();
        // auto leftAngle = encoderData.leftAngle;
        // auto rightAngle = encoderData.rightAngle;
        // float encoderTimestamp = encoderData.timestamp;

        // float circumfrance = 1.048;
        // float totalWheelMeters = 0.0;

        // if (first)
        // {
        //     lastLeft = leftAngle;
        //     lastRight = rightAngle;
        //     first = false;
        // }

        // auto degreesTraveledLeft = degreesTraveled(leftAngle, lastLeft, true);
        // auto degreesTraveledRight = degreesTraveled(rightAngle, lastRight, false);

        // // std::cout << "Degrees traveld Left: " << degreesTraveledLeft << std::endl;

        // // Distance traveled
        // totalDegreesLeft += degreesTraveledLeft;
        // float rotationsLeft = totalDegreesLeft / 360.0;
        // float leftDistanceMeters = rotationsLeft * circumfrance;
        // lastLeft = leftAngle;

        // totalDegreesRight += degreesTraveledRight;
        // float rotationsRight = totalDegreesRight / 360.0;
        // float rightDistanceMeters = rotationsRight * circumfrance;
        // lastRight = rightAngle;

        // totalWheelMeters = (leftDistanceMeters + rightDistanceMeters) / 2;

        // float leftVelocity = leftDiff.differentiate(leftDistanceMeters, encoderTimestamp);
        // float rightVelocity = rightDiff.differentiate(rightDistanceMeters, encoderTimestamp);

        // Point cloud processing
        auto cameraPointData = interfaceNode->luciInterface->getCameraPointCloud();
        // auto radarPointData = interface->luciInterface->getRadarPointCloud();
        // auto ultrasonicPointData = interface->luciInterface->getUltrasonicPointCloud();
        // auto fullPointCloud = cameraPointData + radarPointData + ultrasonicPointData;
        auto fullPointCloud = cameraPointData;
        sensor_msgs::msg::PointCloud2 rosPointCloud;
        pcl::toROSMsg(fullPointCloud, rosPointCloud);
        std_msgs::msg::Header header;
        header.frame_id = "base_camera";
        header.stamp = interfaceNode->currentTime;
        rosPointCloud.header = header;
        interfaceNode->sensorPublisher->publish(rosPointCloud);
        // RCLCPP_ERROR(interfaceNode->get_logger(), "Points: %ld", cameraPointData.size());
        // RCLCPP_ERROR(interfaceNode->get_logger(), std::to_string(cameraPointData.size()));

        // std::cout << "Points : " << cameraPointData.size() << std::endl;

        // AHRS data processing
        auto ahrsData = interfaceNode->luciInterface->getAhrsData();

        // Odometry for navigation / mapping

        // Y axis for luci is x axis in ros system, positive rotation is turning left on ros
        // float vx = ((rightVelocity + leftVelocity) / 2);
        // float vy = 0.0;
        // float vth = ((rightVelocity - leftVelocity) / 0.5334);

        // std::cout << "VX: " << vx << " VTH: " << vth << std::endl;
        // NOT SURE ABOUT MIXING TIME STAMPS HERE?? (ENCODER STREAM VS ROS)

        // double dt = (interfaceNode->currentTime - interfaceNode->lastTime).toSec();
        double dt = 1.0;

        // double deltaX = (ahrsData.linear_velocity.x * cos(interface->th) -
        // ahrsData.linear_velocity.y * sin(interface->th)) * dt; double deltaY =
        // (ahrsData.linear_velocity.x * sin(interface->th) + ahrsData.linear_velocity.y *
        // cos(interface->th)) * dt; double deltaTh = ahrsData.angular_velocity.z * dt;
        // double deltaX = (vx * cos(interfaceNode->th) - vy * sin(interfaceNode->th)) * dt;
        // double deltaY = (vx * sin(interfaceNode->th) + vy * cos(interfaceNode->th)) * dt;
        // double deltaTh = vth * dt;

        // interfaceNode->x += deltaX;
        // interfaceNode->y += deltaY;
        // interfaceNode->th += deltaTh;

        // PID loop for driving from nav
        // geometry_msgs::msg::Twist pidUpdateMsg;
        // pidUpdateMsg.linear.y = vy;
        // pidUpdateMsg.linear.x = vx;
        // pidUpdateMsg.angular.z = vth;
        // interfaceNode->pidPublisher->publish(pidUpdateMsg);

        // geometry_msgs::msg::Quaternion odomQuat = createQuaternionMsgFromYaw(interfaceNode->th);
        // geometry_msgs::msg::TransformStamped odomTrans;
        // odomTrans.header.stamp = interfaceNode->currentTime;
        // odomTrans.header.frame_id = "odom";
        // odomTrans.child_frame_id = "base_link";

        // odomTrans.transform.translation.x = interfaceNode->x;
        // odomTrans.transform.translation.y = interfaceNode->y;
        // odomTrans.transform.translation.z = 0.0;
        // odomTrans.transform.rotation = odomQuat;

        // // Send transform to tf
        // interfaceNode->odomBroadcaster->sendTransform(odomTrans);

        // nav_msgs::msg::Odometry odom;
        // // odom.header.stamp = interface->currentTime;
        // odom.header.frame_id = "odom";

        // // Set position
        // odom.pose.pose.position.x = interfaceNode->x;
        // odom.pose.pose.position.y = interfaceNode->y;
        // odom.pose.pose.position.z = 0.0;
        // odom.pose.pose.orientation = odomQuat;

        // // Set Velocity
        // odom.child_frame_id = "base_link";
        // odom.twist.twist.linear.x = vx;
        // odom.twist.twist.linear.y = vy;
        // odom.twist.twist.angular.z = vth;

        // interfaceNode->odomPublisher->publish(odom);
        // interface->lastTime = interface->currentTime;

        rclcpp::spin_some(interfaceNode);
        loop_rate.sleep();
    }

    // interface->run();

    // ros::NodeHandle node_handle;
    // ros::Subscriber subscriber = node_handle.subscribe("joystick_topic", 1, &sendJSCallback);
    // ros::spin();
    // grpcThreads.emplace_back(&ClientGuide::readJoystickPosition, luciInterface);
    // grpcThreads.emplace_back(&ClientGuide::readChairSpeed, luciInterface);

    // while (true)
    // {
    //     std::cout << "Running..." << std::endl;

    //     // Get position data
    //     float position = luciInterface->getJoystickPosition();
    //     std::cout << position << std::endl;

    //     std::this_thread::sleep_for(std::chrono::seconds(1));
    // }
    // //   activateEngagedMode();

    return 0;
}