#include "../include/interface.h"
#include "../include/client.h"
#include "../include/differentiator.h"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "spdlog/sinks/stdout_color_sinks.h"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/spdlog.h>

void createLogger()
{
    try
    {
        auto stdoutLogger = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
        stdoutLogger->set_level(spdlog::level::info);

        auto fileLogger =
            std::make_shared<spdlog::sinks::basic_file_sink_mt>("ROS-gRPC-interface.txt", true);
        fileLogger->set_level(spdlog::level::warn);

        auto logger =
            std::make_shared<spdlog::logger>("logger", spdlog::sinks_init_list{stdoutLogger});
        spdlog::register_logger(logger);
        spdlog::set_default_logger(logger);
        spdlog::flush_every(std::chrono::seconds(1));
        spdlog::info("Logger Started");
    }
    catch (const std::system_error& e)
    {
        spdlog::error("{}", e.what());
    }
}

void Interface::sendJSCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg)
{
    spdlog::info(" Recieved js val: {} {}", msg->forward_back, msg->left_right);

    this->luciInterface->sendJS(msg->forward_back + 100, msg->left_right + 100);
}

int main(int argc, char** argv)
{
    spdlog::info("Starting...");
    rclcpp::init(argc, argv);
    createLogger();

    std::string host = "localhost";
    std::string port = "50051";
    int opt;
    bool calibrate = false;
    while ((opt = getopt(argc, argv, "a:p:h")) != EOF)
        switch (opt)
        {
        case 'a':
            host = optarg;
            spdlog::info("Using host: {}", host);
            break;
        case 'p':
            port = optarg;
            spdlog::info("Using port: {}", port);
            break;
        case 'h':
        default:
            spdlog::info("-a <host address>     The host address. Standard mode only.");
            spdlog::info("-p <port>             Port number. Standard mode only.");
            spdlog::info("-h                    Show Help.");
            exit(0);
        }

    // TODO: figure out spin rate stuff
    auto interfaceNode = std::make_shared<Interface>(host, port);
    spdlog::info("Connection started at {}:{}", host, port);

    rclcpp::Rate loop_rate(20);

    interfaceNode->luciInterface->activateAutoMode();

    while (rclcpp::ok())
    {
        RCLCPP_INFO(interfaceNode->get_logger(), "Running...");

        // Camera point cloud processing
        auto cameraData = interfaceNode->cameraDataBuff->waitNext();
        auto cameraPointCloud = cameraData;
        spdlog::info("Number of camera points: {}", cameraPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosCameraPointCloud;
        pcl::toROSMsg(cameraPointCloud, rosCameraPointCloud);
        std_msgs::msg::Header cameraHeader;
        cameraHeader.frame_id = "base_camera";
        cameraHeader.stamp = interfaceNode->currentTime;
        rosCameraPointCloud.header = cameraHeader;
        interfaceNode->cameraPublisher->publish(rosCameraPointCloud);

        // Radar point cloud processing
        auto radarData = interfaceNode->radarDataBuff->waitNext();
        auto radarPointCloud = radarData;
        spdlog::info("Number of radar points: {}", radarPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosRadarPointCloud;
        pcl::toROSMsg(radarPointCloud, rosRadarPointCloud);
        std_msgs::msg::Header radarHeader;
        radarHeader.frame_id = "base_radar";
        radarHeader.stamp = interfaceNode->currentTime;
        rosRadarPointCloud.header = radarHeader;
        interfaceNode->radarPublisher->publish(rosRadarPointCloud);

        // Ultrasonic point cloud processing
        auto ultrasonicData = interfaceNode->ultrasonicDataBuff->waitNext();
        auto ultrasonicPointCloud = ultrasonicData;
        spdlog::info("Number of ultrasonic points: {}", ultrasonicPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosUltrasonicPointCloud;
        pcl::toROSMsg(ultrasonicPointCloud, rosUltrasonicPointCloud);
        std_msgs::msg::Header ultrasonicHeader;
        ultrasonicHeader.frame_id = "base_ultrasonic";
        ultrasonicHeader.stamp = interfaceNode->currentTime;
        rosUltrasonicPointCloud.header = ultrasonicHeader;
        interfaceNode->ultrasonicPublisher->publish(rosUltrasonicPointCloud);

        // LUCI Zone Scaling
        auto zoneScalingData = interfaceNode->zoneScalingDataBuff->waitNext();

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

        interfaceNode->zoneScalingPublisher->publish(zoneScalingMsg);

        // LUCI Joystick Scaling
        auto joystickScalingData = interfaceNode->joystickScalingDataBuff->waitNext();

        luci_messages::msg::LuciJoystickScaling joystickScalingMsg;
        joystickScalingMsg.forward_back = joystickScalingData.forward_back;
        joystickScalingMsg.left_right = joystickScalingData.left_right;
        joystickScalingMsg.joystick_zone = joystickScalingData.joystick_zone;

        interfaceNode->joystickScalingPublisher->publish(joystickScalingMsg);

        rclcpp::spin_some(interfaceNode);
        loop_rate.sleep();
    }
    return 0;
}
