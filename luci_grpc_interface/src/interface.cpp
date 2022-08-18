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
        RCLCPP_ERROR(interfaceNode->get_logger(), "Running...");

        // Point cloud processing
        auto cameraPointData = interfaceNode->cameraPointsDataBuff->waitNext();
        auto radarPointData = interfaceNode->radarPointsDataBuff->getLatest();
        auto ultrasonicData = interfaceNode->ultrasonicDataBuff->getLatest();

        auto fullPointCloud = cameraPointData;
        spdlog::error("Points: {}", fullPointCloud.size());
        sensor_msgs::msg::PointCloud2 rosPointCloud;
        pcl::toROSMsg(fullPointCloud, rosPointCloud);
        std_msgs::msg::Header header;
        header.frame_id = "base_camera";
        header.stamp = interfaceNode->currentTime;
        rosPointCloud.header = header;
        interfaceNode->sensorPublisher->publish(rosPointCloud);

        rclcpp::spin_some(interfaceNode);
        loop_rate.sleep();
    }

    return 0;
}