cmake_minimum_required(VERSION 3.0.2)
project(grpc_interface)

set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(PCL 1.3 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    translator
    tf
)

catkin_package()

add_executable(grpc_interface_node 
    src/interface.cpp  
    src/differentiator.cpp
    include/interface.h
    include/differentiator.h  
    ./generated_code/client_library/ptolemy.grpc.pb.cc
    ./generated_code/client_library/ptolemy.pb.cc
)

add_dependencies(grpc_interface_node ${catkin_EXPORTED_TARGETS})

find_package(protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(OpenCV CONFIG REQUIRED)
# find_library(protobuf NAMES protobuf)
find_library(OpenCV NAMES OpenCV)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_include_directories(grpc_interface_node PUBLIC ./generated_code)

target_link_libraries(grpc_interface_node PUBLIC ${catkin_LIBRARIES} 
                        # protobuf 
                        gRPC::grpc++_unsecure
                        ${OpenCV_LIBS})

target_include_directories(grpc_interface_node PUBLIC
                        ${CMAKE_CURRENT_SOURCE_DIR}/include)
