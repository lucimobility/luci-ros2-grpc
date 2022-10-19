#include "client.h"
#include <iostream>
#include <memory>

#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/transform_broadcaster.h>

#include <luci_messages/msg/luci_joystick.hpp>
#include <luci_messages/msg/luci_scaling.hpp>
#include <std_msgs/msg/string.hpp>
#include <tf2_ros/transform_broadcaster.h>

using std::placeholders::_1;

/// Time (seconds) to timeout on attempt to connect to grpc
constexpr auto GRPC_CHANNEL_TIMEOUT = std::chrono::seconds(5);

class Interface : public rclcpp::Node
{
  private:
    rclcpp::Subscription<luci_messages::msg::LuciJoystick>::SharedPtr subscription_;

  public:
    std::shared_ptr<grpc::Channel> grpcChannel;

    std::chrono::time_point<std::chrono::system_clock> deadline =
        std::chrono::system_clock::now() + GRPC_CHANNEL_TIMEOUT;

    std::shared_ptr<Luci::ROS2::DataBuffer<SystemJoystick>> joystickDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<SystemJoystick>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<float>> chairSpeedDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<float>>();

    std::shared_ptr<Luci::ROS2::DataBuffer<LuciScaling>> scalingDataBuff =
        std::make_shared<Luci::ROS2::DataBuffer<LuciScaling>>();

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
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pidPublisher;
    rclcpp::Publisher<luci_messages::msg::LuciScaling>::SharedPtr scalingPublisher;
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
            chairSpeedDataBuff, scalingDataBuff);

        subscription_ = this->create_subscription<luci_messages::msg::LuciJoystick>(
            "joystick_topic", 1,
            [this](luci_messages::msg::LuciJoystick::SharedPtr msg) { sendJSCallback(msg); });

        pidPublisher = this->create_publisher<geometry_msgs::msg::Twist>("chair/cmd_vel", 1);

        scalingPublisher =
            this->create_publisher<luci_messages::msg::LuciScaling>("luci_scaling", 1);

        cameraPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("camera_cloud_in", 1);
        radarPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("radar_cloud_in", 1);
        ultrasonicPublisher =
            this->create_publisher<sensor_msgs::msg::PointCloud2>("ultrasonic_cloud_in", 1);
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        odomBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    }

    rclcpp::Time currentTime, lastTime;

    void sendJSCallback(const luci_messages::msg::LuciJoystick::SharedPtr msg);
    void run();

    ~Interface();
};

Interface::~Interface() {}
