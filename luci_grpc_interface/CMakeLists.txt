cmake_minimum_required(VERSION 3.16)
# Project name must match the package name in package.xml
project(luci_grpc_interface)

# Set standards
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set thread library
set(THREADS_PREFER_PTHREAD_FLAG ON)

# Find ROS2 cmake macros
find_package(ament_cmake REQUIRED)

# Add subdirs for libraries and executables
add_subdirectory(third_party/tclap)
add_subdirectory(sensors_grpc)
add_subdirectory(client)
add_subdirectory(interface)

# Package the ROS2 node in a ROS2 compatible way
ament_package()
