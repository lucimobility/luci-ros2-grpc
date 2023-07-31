# Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.

# Make a GrpcClient library for use in ROS2 package
add_library(GrpcClient src/client.cpp)

# Find all needed libraries
find_package(protobuf NAMES protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(PCL REQUIRED)
find_package(fmt CONFIG REQUIRED)

# Add include dir to be exposed in the library when built
target_include_directories(GrpcClient PUBLIC 
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link all needed libraries into the client (Note the links are static and a
# person running the ROS package will NOT need gRPC installed)
target_link_libraries(GrpcClient PUBLIC 
    SensorsGrpc 
    protobuf 
    gRPC::grpc++_unsecure 
    fmt::fmt
    ${PCL_LIBRARIES}
)
