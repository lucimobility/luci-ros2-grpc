#include "../include/interface.h"
#include "../include/client.h"
#include "../include/differentiator.h"
#include <luci_messages/msg/luci_joystick.h>

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
        spdlog::warn("Logger Started");
    }
    catch (const std::system_error& e)
    {
        spdlog::error("{}", e.what());
    }
}

void Interface::sendJSCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg)
{
    spdlog::warn(" Recieved js val: {} {}", msg->forward_back, msg->left_right);

    this->luciInterface->sendJS(msg->forward_back + 100, msg->left_right + 100);
}

int main(int argc, char** argv)
{
    spdlog::error("Starting...");
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
            spdlog::warn("Using host: {}", host);
            break;
        case 'p':
            port = optarg;
            spdlog::warn("Using port: {}", port);
            break;
        case 'h':
        default:
            spdlog::warn("-a <host address>     The host address. Standard mode only.");
            spdlog::warn("-p <port>             Port number. Standard mode only.");
            spdlog::warn("-h                    Show Help.");
            exit(0);
        }

    // TODO: figure out spin rate stuff
    auto interfaceNode = std::make_shared<Interface>(host, port);
    spdlog::warn("Connection started at {}:{}", host, port);

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

        rclcpp::spin_some(interfaceNode);
        loop_rate.sleep();
    }
    return 0;
}
