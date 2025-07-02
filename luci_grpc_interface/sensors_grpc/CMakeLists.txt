# Copyright 2025 LUCI Mobility, Inc
#
#  Licensed under the Apache License, Version 2.0 (the "License");
#  you may not use this file except in compliance with the License.
#  You may obtain a copy of the License at
# 
#     https://www.apache.org/licenses/LICENSE-2.0
# 
#  Unless required by applicable law or agreed to in writing, software
#  distributed under the License is distributed on an "AS IS" BASIS,
#  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
#  See the License for the specific language governing permissions and
#  limitations under the License.

# Generate the protobuf files (pulled from sensors repo)
execute_process(
  COMMAND "${CMAKE_CURRENT_SOURCE_DIR}/gen-proto.sh" ${CMAKE_CURRENT_SOURCE_DIR}
          ${CMAKE_BINARY_DIR} ${CMAKE_CURRENT_BINARY_DIR}
  RESULT_VARIABLE GRPC_COMMAND_RESULT
  ERROR_VARIABLE GRPC_ERROR_MESSAGE)

set(GRPC_COMMAND_RESULT FALSE)

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
