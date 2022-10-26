# README #

## How to build and run this package ##

move into the directory: /luci_ros_grpc/luci_grpc_interface/

source /opt/ros/galactic/setup.sh
colcon build
source install/setup.bash
ros2 run luci_grpc_interface grpc_interface_node    
