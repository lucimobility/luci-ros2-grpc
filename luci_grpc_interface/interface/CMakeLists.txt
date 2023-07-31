# Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.

# Find all the needed libraries for ROS
find_package(ament_cmake REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(luci_messages REQUIRED)

# Make a gRPC<-> ROS2 library
add_library(ROS2GrpcInterfaceNode
    src/interface.cpp
)

# Add all the .h files in the include dir
target_include_directories(ROS2GrpcInterfaceNode PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${pcl_conversions_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${nav_msgs_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${luci_messages_INCLUDE_DIRS}
)

# Link all the libraries 
target_link_libraries(ROS2GrpcInterfaceNode PUBLIC 
    rclcpp::rclcpp 
    sensor_msgs::sensor_msgs_library
    GrpcClient
    tclap
    ${pcl_conversions_LIBRARIES}
    ${nav_msgs_LIBRARIES}
    ${geometry_msgs_LIBRARIES}
    ${tf2_ros_LIBRARIES}
    ${luci_messages_LIBRARIES}
)

# Make the interface node executable
add_executable(grpc_interface_node 
    src/main.cpp
)

# Link the libraries to the executable
target_link_libraries(grpc_interface_node ROS2GrpcInterfaceNode)

# Install to ROS expected folder in the intall dir
install (TARGETS grpc_interface_node DESTINATION lib/${PROJECT_NAME})
