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
// #include <translator/luci_joystick.h>
#include <std_msgs/msg/string.hpp>
using std::placeholders::_1;

class Interface : public rclcpp::Node
{
  private:
    // std::string host = "192.168.8.125";
    // std::string host = "10.1.10.115";
    // std::string host = "10.1.10.182";
    // std::string port = "50051";

    // std::vector<std::thread> grpcThreads;

    // std::shared_ptr<rclcpp::Node> node =
    // rclcpp::Node::make_shared("interface");
    // rclcpp::Subscription<std_msgs::msg::String> subscriber =
    //     node->create_subscription<std_msgs::msg::String>(
    //         "joystick_topic", 1, std::bind(&Interface::sendJSCallback, this,
    //         _1));

    // std::shared_ptr<rclcpp::Subscription<std_msgs::msg::String>> subscriber =
    // node->create_subscription<std_msgs::msg::String>("joystick_topic", 1,
    // std::bind(&Interface::sendJSCallback, this, _1));

    // rclcpp::AsyncSpinner spinner;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;

  public:
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sensorPublisher;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pidPublisher;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odomPublisher;
    std::shared_ptr<tf2_ros::TransformBroadcaster> odomBroadcaster;

    // ClientGuide* luciInterface =
    //     new ClientGuide(grpc::CreateChannel(host + ":" + port, grpc::InsecureChannelCredentials()));

    Interface() : Node("interface")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "joystick_topic", 1,
            [this](std_msgs::msg::String::SharedPtr msg) { sendJSCallback(msg); });

        // subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/camera/depth/color/points", 10,
        //     [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) { topic_callback(msg); });

        pidPublisher = this->create_publisher<geometry_msgs::msg::Twist>("chair/cmd_vel", 1);

        sensorPublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 1);
        odomPublisher = this->create_publisher<nav_msgs::msg::Odometry>("odom", 1);
        odomBroadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        // grpcThreads.emplace_back(&ClientGuide::readAhrsData, luciInterface);
        // grpcThreads.emplace_back(&ClientGuide::readCameraPointData, luciInterface);
        // // grpcThreads.emplace_back(&ClientGuide::readEncoderData, luciInterface);
    }

    // rclcpp::Publisher<geometry_msgs::msg::Twist> pidPublisher =
    // node->create_publisher<geometry_msgs::msg::Twist>("chair/cmd_vel", 1);
    // rclcpp::Publisher::SharedPtr pidPublisher =
    // node->advertise<geometry_msgs::msg::Twist>("chair/cmd_vel", 1);

    // rclcpp::Publisher<sensor_msgs::msg::PointCloud2> sensorPublisher =
    //     node->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_in", 1);
    // rclcpp::Publisher<nav_msgs::msg::Odometry> odomPublisher =
    //     node->create_publisher<nav_msgs::msg::Odometry>("odom", 1);

    // Positions for odom
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    rclcpp::Time currentTime, lastTime;

    // RCLCPP_ERROR(this->get_logger(), "connection established");

    // void sendJSCallback(const translator::luci_joystickConstPtr& msg);
    void sendJSCallback(const std_msgs::msg::String::SharedPtr msg);
    void run();
    // Interface();

    ~Interface();
};

// Interface::Interface(/* args */) {}

Interface::~Interface() {}