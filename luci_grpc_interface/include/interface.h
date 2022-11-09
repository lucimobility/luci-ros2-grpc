#include "client.h"
#include <iostream>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_with_covariance.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <luci_messages/msg/luci_joystick.hpp>
#include <luci_messages/msg/luci_joystick_scaling.hpp>
#include <luci_messages/msg/luci_zone_scaling.hpp>

#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

/// Time (seconds) to timeout on attempt to connect to grpc
constexpr auto GRPC_CHANNEL_TIMEOUT = std::chrono::seconds(5);

class Interface : public rclcpp::Node
{
  private:
    rclcpp::Subscription<luci_messages::msg::LuciJoystick>::SharedPtr remote_js_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr luci_mode_switch_subscription_;


  public:
    std::shared_ptr<grpc::Channel> grpcChannel;

    std::chrono::time_point<std::chrono::system_clock> deadline =
        std::chrono::system_clock::now() + GRPC_CHANNEL_TIMEOUT;

    std::shared_ptr<Luci::ROS2::DataBuffer<SystemJoystick>> joystickDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<SystemJoystick>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<LuciZoneScaling>> zoneScalingDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<LuciZoneScaling>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<LuciJoystickScaling>> joystickScalingDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<LuciJoystickScaling>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<AhrsInfo>> ahrsInfoBuff =
        std::make_shared<Luci::ROS2::DataBuffer<AhrsInfo>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> cameraDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> radarDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>> ultrasonicDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<pcl::PointCloud<pcl::PointXYZ>>>();

    std::shared_ptr<Luci::ROS2::ClientGuide> luciInterface;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cameraPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr radarPublisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr ultrasonicPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciZoneScaling>::SharedPtr zoneScalingPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciJoystickScaling>::SharedPtr joystickScalingPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciJoystick>::SharedPtr joystickPositionPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster;

    Interface(std::string host, std::string port = "50051") : Node("interface")
    {

        grpcChannel = grpc::CreateChannel(host + ":" + port, grpc::InsecureChannelCredentials());

        std::chrono::time_point<std::chrono::system_clock> deadline =
            std::chrono::system_clock::now() + GRPC_CHANNEL_TIMEOUT;

        bool connected = grpcChannel->WaitForConnected(deadline);

        luciInterface = std::make_shared<Luci::ROS2::ClientGuide>(
            grpcChannel, joystickDataBuff, cameraDataBuff, radarDataBuff, ultrasonicDataBuff,
            zoneScalingDataBuff, joystickScalingDataBuff, ahrsInfoBuff);

        remote_js_subscription_ = this->create_subscription<luci_messages::msg::LuciJoystick>(
            "luci/remote_joystick", 1,
            [this](luci_messages::msg::LuciJoystick::SharedPtr msg) { sendJSCallback(msg); });

        luci_mode_switch_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "luci/drive_mode", 1,
            [this](std_msgs::msg::String::SharedPtr msg) { switchLuciModeCallback(msg); });

        zoneScalingPublisher =
            this->create_publisher<luci_messages::msg::LuciZoneScaling>("luci/scaling", 1);

        joystickScalingPublisher = this->create_publisher<luci_messages::msg::LuciJoystickScaling>(
            "luci/joystick_scaling", 1);

        joystickPositionPublisher =
            this->create_publisher<luci_messages::msg::LuciJoystick>("luci/joystick_position", 1);

        cameraPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("luci/camera_points", 1);
        radarPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("luci/radar_points", 1);
        ultrasonicPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("luci/ultrasonic_points", 1);
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("luci/odom", 1);
        odomBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    rclcpp::Time currentTime, lastTime;

    void sendJSCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg);
    void switchLuciModeCallback(const std_msgs::msg::String::SharedPtr msg);

    void run();

    ~Interface();
};

Interface::~Interface() {}
