#!/bin/bash

set -e

USAGE="Usage: gen-proto.sh CMAKE_CURRENT_SOURCE_DIR CMAKE_BINARY_DIR CMAKE_CURRENT_BINARY_DIR"

if [[ ${1} == "-h" || ${1} == "--help" ]]; then
    echo ${USAGE}
    exit 0
fi

if [[ -z ${1} ]]; then
    echo >&2 "Missing CMAKE_CURRENT_SOURCE_DIR operand"
    echo >&2 ${USAGE}
    exit 1
fi

if [[ -z ${2} ]]; then
    echo >&2 "Missing CMAKE_BINARY_DIR operand"
    echo >&2 ${USAGE}
    exit 1
fi

if [[ -z ${3} ]]; then
    echo >&2 "Missing CMAKE_CURRENT_BINARY_DIR operand"
    echo >&2 ${USAGE}
    exit 1
fi

CMAKE_CURRENT_SOURCE_DIR=${1}
CMAKE_BINARY_DIR=${2}
CMAKE_CURRENT_BINARY_DIR=${3}

# Directory to search for imports
PROTO_PATH=${CMAKE_CURRENT_SOURCE_DIR}
PROTOFILE_PATH=${CMAKE_CURRENT_SOURCE_DIR}/sensors.proto

PROTOC=protoc
CPP_PLUGIN=grpc_cpp_plugin

CONAN_PROTOC=${CMAKE_BINARY_DIR}/bin/${PROTOC}
CONAN_CPP_PLUGIN=${CMAKE_BINARY_DIR}/bin/${CPP_PLUGIN}

if [[ -x ${CONAN_PROTOC} ]]; then
    PROTOC=${CONAN_PROTOC}
fi

if [[ -x ${CONAN_CPP_PLUGIN} ]]; then
    CPP_PLUGIN=${CONAN_CPP_PLUGIN}
else
    # Protoc does not search the PATH when looking for plugins,
    # so we must provide the full path.
    CPP_PLUGIN=$(which ${CPP_PLUGIN})
    echo $CPP_PLUGIN
fi

echo "PROTOC=${PROTOC}"
echo "CPP_PLUGIN=${CPP_PLUGIN}"

PLUGIN=protoc-gen-grpc=${CPP_PLUGIN}
OUTPUT_DIR=${CMAKE_CURRENT_BINARY_DIR}/generated_code/sensors_grpc
HASH_FILE=${OUTPUT_DIR}/proto-hash.md5

mkdir -p ${OUTPUT_DIR}

if md5sum --check ${HASH_FILE}; then
    echo "Skipping protobuf generation"
    exit 0
else
    echo "Proto change detected. Generating proto files."
fi

${PROTOC} \
    --proto_path=${PROTO_PATH} \
    --cpp_out=${OUTPUT_DIR} \
    ${PROTOFILE_PATH}
echo "Proto files written"

${PROTOC} \
    --proto_path=${PROTO_PATH} \
    --grpc_out=${OUTPUT_DIR} \
    --plugin=${PLUGIN} \
    ${PROTOFILE_PATH}
echo "gRPC files written"

echo -n $(md5sum ${PROTOFILE_PATH}) > ${HASH_FILE}
echo "Hash stored"
