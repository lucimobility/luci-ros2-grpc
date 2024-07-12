/**
 * @file data_buffer.h
 * @brief Definition of the sharable data buffers, each stream from client connection gets its own
 * data buffer
 *
 * @copyright Copyright 2024 LUCI Mobility, Inc
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *    https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "closed_exception.h"

#include <condition_variable>
#include <list>
#include <mutex>
#include <optional>

#include <spdlog/spdlog.h>

namespace Luci::ROS2
{

/**
 * @brief Data buffer object
 *
 * @tparam T Type of data the buffer will hold (CameraFrame, SystemSpeed, etc)
 */
template <typename T> class DataBuffer
{
  public:
    /**
     * @brief Add data to the buffer, only called from single producer
     *
     * @param data
     */
    void push(T data)
    {
        {
            std::scoped_lock dataLock(this->_dataMutex);
            this->_data = data;

            for (auto& it : this->_waiters)
            {
                it = true;
            }
        }
        /// Inform all waiting consumers there is new data available
        this->_conditionVariable.notify_all();
    }

    /**
     * @brief Get the Latest object from the data buffer
     *
     * @return std::optional<T>
     */
    std::optional<T> getLatest()
    {
        std::scoped_lock dataLock(_dataMutex);
        return _data;
    }

    /**
     * @brief Wait until new data appears on the buffer
     * No timeout will wait forever
     *
     * @return T
     */
    T waitNext()
    {
        std::unique_lock dataLock(this->_dataMutex);
        auto it = this->_waiters.insert(this->_waiters.end(), false);
        this->_conditionVariable.wait(dataLock, [it, this] { return *it || !isOpen; });

        if (!isOpen)
        {
            // Throw if it is closed.
            throw 20;
        }

        this->_waiters.erase(it);
        return *(this->_data);
    }

    /**
     * @brief Wait until new data appears on the buffer or we timeout
     *
     * @param waitTime
     * @return std::optional<T>
     */
    std::optional<T> waitNext(std::chrono::milliseconds waitTime)
    {
        std::unique_lock dataLock(this->_dataMutex);
        auto it = this->_waiters.insert(this->_waiters.end(), false);

        auto dataReceived = this->_conditionVariable.wait_for(
            dataLock, waitTime, [it, this] { return *it || !isOpen; });

        if (!isOpen)
        {
            throw 20;
        }

        this->_waiters.erase(it);
        if (dataReceived)
        {
            return _data;
        }

        return {};
    }

    void close()
    {
        {
            std::scoped_lock dataLock(_dataMutex);
            isOpen = false;
        }
        _conditionVariable.notify_all();
    }

  private:
    /// List representing consumers waiting on new data
    std::list<bool> _waiters;

    /// Data stored on the buffer
    std::optional<T> _data = std::nullopt;

    /// Condition variable for notification
    std::condition_variable _conditionVariable;

    /// Data mutex lock
    mutable std::mutex _dataMutex;

    /// Flag indicating whether the data buffer is open. It starts open, and then can manually be
    /// closed.
    bool isOpen = true;
};
} // namespace Luci::ROS2
