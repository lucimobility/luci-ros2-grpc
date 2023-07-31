# Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.

# Generate the protobuf files (pulled from sensors repo)
execute_process(
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/gen-proto.sh" ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE GRPC_COMMAND_RESULT
  ERROR_VARIABLE GRPC_ERROR_MESSAGE)

if(${GRPC_COMMAND_RESULT})
  message(
    FATAL_ERROR
      "Unable to generate gRPC files. Aborting build.\n\nOutput:\n\n${GRPC_ERROR_MESSAGE}"
  )
else(${GRPC_COMMAND_RESULT})
  message(STATUS "Successfully generated gRPC files:\n${GRPC_ERROR_MESSAGE}")
endif(${GRPC_COMMAND_RESULT})

set(SRC_GRPC
    ${CMAKE_CURRENT_BINARY_DIR}/generated_code/sensors_grpc/sensors.grpc.pb.cc
    ${CMAKE_CURRENT_BINARY_DIR}/generated_code/sensors_grpc/sensors.pb.cc)

add_library(SensorsGrpc ${SRC_GRPC})

# Because the docker image used grpc to install protobuf you need to find
# protobuf before you call find grpc
find_package(protobuf NAMES protobuf REQUIRED)
find_package(gRPC CONFIG REQUIRED)

target_link_libraries(SensorsGrpc PRIVATE 
    protobuf 
    gRPC::grpc++_unsecure 
)

source_group("gRPC" FILES ${SRC_GRPC})

target_include_directories(SensorsGrpc PUBLIC
    ${CMAKE_CURRENT_BINARY_DIR}/generated_code 
)
