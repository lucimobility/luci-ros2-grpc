#include "../include/interface.h"
#include "../include/differentiator.h"
std::vector<std::thread> processThreads;

// #include <spdlog/spdlog.h>

// ClientGuide* luciInterface = new ClientGuide(grpc::CreateChannel(host + ":" + port, grpc::InsecureChannelCredentials()));
// std::cout << "Client created at " << host << port << std::endl;

void Interface::sendJSCallback(const translator::luci_joystickConstPtr& msg)
{
    // std::cout << "Recieved js val: " << msg->forwardBack << " " << msg->leftRight << std::endl;

    this->luciInterface->sendJS(msg->forwardBack + 100, msg->leftRight + 100);
}

Interface::Interface() : spinner(4)
{
    grpcThreads.emplace_back(&ClientGuide::readAhrsData, luciInterface);
    grpcThreads.emplace_back(&ClientGuide::readCameraPointData, luciInterface);
    // grpcThreads.emplace_back(&ClientGuide::readRadarPointData, luciInterface);
    // grpcThreads.emplace_back(&ClientGuide::readUltrasonicPointData, luciInterface);

    grpcThreads.emplace_back(&ClientGuide::readEncoderData, luciInterface);

    spinner.start();
}

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

int main(int argc, char** argv)
{

    ros::init(argc, argv, "interface");

    Interface* interface = new Interface();
    ros::Rate rate(20);


    // CHECK THE POINTCLOUD TO MAKE SURE ITS RIGHT WAY UP
    // interface->luciInterface->activateAutoMode();
    interface->luciInterface->activateUserMode();

    // processThreads.emplace_back(&Interface::run, interface);

    // Encoder stuff
    Differentiator leftDiff;
    Differentiator rightDiff;
    bool first = true;
    float lastLeft = 0.0;
    float lastRight = 0.0;

    float totalDegreesLeft = 0.0;
    float totalDegreesRight = 0.0;

    interface->currentTime = ros::Time::now();
    interface->lastTime = ros::Time::now();

    while (ros::ok())
    {
        interface->currentTime = ros::Time::now();

        auto encoderData = interface->luciInterface->getEncoderData();
        auto leftAngle = encoderData.leftAngle;
        auto rightAngle = encoderData.rightAngle;
        float encoderTimestamp = encoderData.timestamp;

        float circumfrance = 1.048;
        float totalWheelMeters = 0.0;

        if (first)
        {
            lastLeft = leftAngle;
            lastRight = rightAngle;
            first = false;
        }

        auto degreesTraveledLeft = degreesTraveled(leftAngle, lastLeft, true);
        auto degreesTraveledRight = degreesTraveled(rightAngle, lastRight, false);

        // std::cout << "Degrees traveld Left: " << degreesTraveledLeft << std::endl;

        // Distance traveled
        totalDegreesLeft += degreesTraveledLeft;
        float rotationsLeft = totalDegreesLeft / 360.0;
        float leftDistanceMeters = rotationsLeft * circumfrance;
        lastLeft = leftAngle;

        totalDegreesRight += degreesTraveledRight;
        float rotationsRight = totalDegreesRight / 360.0;
        float rightDistanceMeters = rotationsRight * circumfrance;
        lastRight = rightAngle;

        // totalWheelMeters = (leftDistanceMeters + rightDistanceMeters) / 2;

        float leftVelocity = leftDiff.differentiate(leftDistanceMeters, encoderTimestamp);
        float rightVelocity = rightDiff.differentiate(rightDistanceMeters, encoderTimestamp);

        // Point cloud processing
        auto cameraPointData = interface->luciInterface->getCameraPointCloud();
        // auto radarPointData = interface->luciInterface->getRadarPointCloud();
        // auto ultrasonicPointData = interface->luciInterface->getUltrasonicPointCloud();
        // auto fullPointCloud = cameraPointData + radarPointData + ultrasonicPointData;
        auto fullPointCloud = cameraPointData;
        sensor_msgs::PointCloud2 rosPointCloud;
        pcl::toROSMsg(fullPointCloud, rosPointCloud);
        std_msgs::Header header;
        header.frame_id = "base_camera";
        header.stamp = interface->currentTime;
        rosPointCloud.header = header;
        interface->sensorPublisher.publish(rosPointCloud);
        // std::cout << "Points : " << pointData.size() << std::endl;

        // AHRS data processing
        auto ahrsData = interface->luciInterface->getAhrsData();

        // Odometry for navigation / mapping

        // Y axis for luci is x axis in ros system, positive rotation is turning left on ros
        float vx = ((rightVelocity + leftVelocity) / 2);
        float vy = 0.0;
        float vth = ((rightVelocity - leftVelocity) / 0.5334);

        // std::cout << "VX: " << vx << " VTH: " << vth << std::endl;
        // NOT SURE ABOUT MIXING TIME STAMPS HERE?? (ENCODER STREAM VS ROS)
        double dt = (interface->currentTime - interface->lastTime).toSec();
        // double deltaX = (ahrsData.linear_velocity.x * cos(interface->th) - ahrsData.linear_velocity.y * sin(interface->th)) * dt;
        // double deltaY = (ahrsData.linear_velocity.x * sin(interface->th) + ahrsData.linear_velocity.y * cos(interface->th)) * dt;
        // double deltaTh = ahrsData.angular_velocity.z * dt;
        double deltaX = (vx * cos(interface->th) - vy * sin(interface->th)) * dt;
        double deltaY = (vx * sin(interface->th) + vy * cos(interface->th)) * dt;
        double deltaTh = vth * dt;

        interface->x += deltaX;
        interface->y += deltaY;
        interface->th += deltaTh;

        // PID loop for driving from nav
        geometry_msgs::Twist pidUpdateMsg;
        pidUpdateMsg.linear.y = vy;
        pidUpdateMsg.linear.x = vx;
        pidUpdateMsg.angular.z = vth;
        interface->pidPublisher.publish(pidUpdateMsg);

        geometry_msgs::Quaternion odomQuat = tf::createQuaternionMsgFromYaw(interface->th);
        geometry_msgs::TransformStamped odomTrans;
        odomTrans.header.stamp = interface->currentTime;
        odomTrans.header.frame_id = "odom";
        odomTrans.child_frame_id = "base_link";

        odomTrans.transform.translation.x = interface->x;
        odomTrans.transform.translation.y = interface->y;
        odomTrans.transform.translation.z = 0.0;
        odomTrans.transform.rotation = odomQuat;

        // Send transform to tf
        interface->odomBroadcaster.sendTransform(odomTrans);

        nav_msgs::Odometry odom;
        odom.header.stamp = interface->currentTime;
        odom.header.frame_id = "odom";

        // Set position
        odom.pose.pose.position.x = interface->x;
        odom.pose.pose.position.y = interface->y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odomQuat;

        // Set Velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        interface->odomPublisher.publish(odom);
        interface->lastTime = interface->currentTime;

        ros::spinOnce();
        rate.sleep();
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