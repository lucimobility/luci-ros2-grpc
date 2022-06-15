#include "client.h"
#include "iostream"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>
#include <translator/luci_joystick.h>

class Interface
{
  private:
    // std::string host = "192.168.8.125";
    // std::string host = "10.1.10.115";
    std::string host = "192.168.8.137";
    std::string port = "50051";

    std::vector<std::thread> grpcThreads;

    ros::NodeHandle node_handle;
    ros::Subscriber subscriber = node_handle.subscribe("joystick_topic", 1, &Interface::sendJSCallback, this);

    ros::AsyncSpinner spinner;

  public:
    ros::Publisher pidPublisher = node_handle.advertise<geometry_msgs::Twist>("chair/cmd_vel", 1);
    ros::Publisher sensorPublisher = node_handle.advertise<sensor_msgs::PointCloud2>("cloud_in", 1);
    ros::Publisher odomPublisher = node_handle.advertise<nav_msgs::Odometry>("odom", 1);

    tf::TransformBroadcaster odomBroadcaster;

    // Positions for odom
    double x = 0.0;
    double y = 0.0;
    double th = 0.0;

    ros::Time currentTime, lastTime;

    ClientGuide* luciInterface = new ClientGuide(grpc::CreateChannel(host + ":" + port, grpc::InsecureChannelCredentials()));

    void sendJSCallback(const translator::luci_joystickConstPtr& msg);
    void run();
    Interface();

    ~Interface();
};

// Interface::Interface(/* args */) {}

Interface::~Interface() {}