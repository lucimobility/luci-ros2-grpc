cmake_minimum_required(VERSION 3.8)
project(luci_grpc_interface)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(PCL REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(luci_messages REQUIRED)
find_package(absl REQUIRED)


find_package(gRPC CONFIG REQUIRED)

find_package(OpenCV CONFIG REQUIRED)
find_package(tf2_ros REQUIRED)
find_library(OpenCV NAMES OpenCV)

add_executable(grpc_interface_node 
    src/interface.cpp  
    src/differentiator.cpp
    src/client.cpp
    include/interface.h
    include/differentiator.h  
    include/client.h
    include/data_buffer.h
    ./generated_code/client_library/sensors.grpc.pb.cc
    ./generated_code/client_library/sensors.pb.cc
)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(grpc_interface_node PUBLIC ./generated_code)

target_link_libraries(grpc_interface_node
                    gRPC::grpc++_unsecure)

ament_target_dependencies(grpc_interface_node "geometry_msgs" "rclcpp"
   "PCL" "OpenCV" "tf2_ros" "nav_msgs" "sensor_msgs" "std_msgs" "luci_messages")


target_include_directories(grpc_interface_node PUBLIC
                        ${CMAKE_CURRENT_SOURCE_DIR}/include)
                        

install(TARGETS
grpc_interface_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
