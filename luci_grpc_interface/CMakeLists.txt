cmake_minimum_required(VERSION 3.8)
project(grpc_interface)


set(CMAKE_LIBRARY_PATH "/home/christian/.local/lib")

# set(CMAKE_LIBRARY_PATH "/home/christian/.conan/data")
# include(${CMAKE_BINARY_DIR}/conanbuildinfo.cmake)
# conan_basic_setup(TARGETS KEEP_RPATHS)

set(THREADS_PREFER_PTHREAD_FLAG ON)

include(FetchContent)

# find_package(Protobuf REQUIRED)

find_package(PCL REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

find_package(gRPC CONFIG REQUIRED)
# FetchContent_MakeAvailable(gRPC)

find_package(OpenCV CONFIG REQUIRED)
find_package(tf2_ros REQUIRED)
# find_library(OpenCV NAMES OpenCV)




# find_package(catkin REQUIRED COMPONENTS
#     roscpp
#     std_msgs
#     translator
#     tf
# )

# catkin_package()

add_executable(grpc_interface_node 
    src/interface.cpp  
    src/differentiator.cpp
    include/interface.h
    include/differentiator.h  
    ./generated_code/client_library/ptolemy.grpc.pb.cc
    ./generated_code/client_library/ptolemy.pb.cc
)

# add_dependencies(grpc_interface_node ${catkin_EXPORTED_TARGETS})


include_directories(
    # ${ament_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    # ${geometry_msgs_INCLUDE_DIRS}
)

target_include_directories(grpc_interface_node PUBLIC ./generated_code)
target_include_directories(grpc_interface_node PUBLIC /home/christian/.local/lib)


target_link_libraries(grpc_interface_node
                    gRPC::grpc++_unsecure)

ament_target_dependencies(grpc_interface_node "geometry_msgs" "rclcpp"
   "PCL")



# target_link_libraries(grpc_interface_node
# #                         # protobuf 
# #                         # ${ament_LIBRARIES}
# #                         # ${common_interfaces_TARGETS}
#                         gRPC::grpc++_unsecure)
# #                         # ${OpenCV_LIBS})

# target_link_libraries(grpc_interface_node
# # CONAN_PKG::spdlog
# CONAN_PKG::opencv
# # CONAN_PKG::librealsense
# CONAN_PKG::grpc
# )

target_include_directories(grpc_interface_node PUBLIC
                        ${CMAKE_CURRENT_SOURCE_DIR}/include)
                        

ament_package()
