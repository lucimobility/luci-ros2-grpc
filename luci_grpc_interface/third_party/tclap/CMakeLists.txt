# Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.

add_library(tclap INTERFACE
)

target_include_directories(tclap INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
