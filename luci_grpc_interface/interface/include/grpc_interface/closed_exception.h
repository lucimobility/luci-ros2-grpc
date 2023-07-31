/**
 * @file closed_exception.h
 *
 * Exception indicating one of the buffer structures has been closed.
 *
 * @copyright Copyright (c) 2023 LUCI Mobility, Inc. All Rights Reserved.
 */

#pragma once

#include <stdexcept>

namespace Luci::ROS2
{
class ClosedException : public std::runtime_error
{
    using std::runtime_error::runtime_error;
};
} // namespace Luci::ROS2