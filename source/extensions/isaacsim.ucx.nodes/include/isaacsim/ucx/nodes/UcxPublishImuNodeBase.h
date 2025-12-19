// SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
// http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <isaacsim/ucx/core/UcxListenerRegistry.h>
#include <isaacsim/ucx/core/UcxUtils.h>
#include <isaacsim/ucx/nodes/UcxNode.h>
#include <omni/graph/core/CppWrappers.h>
#include <omni/graph/core/iComputeGraph.h>

#include <cstring>
#include <vector>

using omni::graph::core::GraphInstanceID;
using omni::graph::core::NodeObj;

/**
 * @class UCXPublishImuNodeBase
 * @brief Templated base class for UCX IMU data publishing nodes.
 * @details
 * This template provides common functionality for publishing IMU sensor data over UCX
 * with timeout support. Derived classes implement message generation logic via generateMessage().
 *
 * @tparam DatabaseT The OGN database type for the node
 */
template <typename DatabaseT>
class UCXPublishImuNodeBase : public isaacsim::ucx::nodes::UcxNode
{
public:
    /**
     * @brief Initialize the node instance.
     * @details
     * Default implementation - can be overridden by derived classes if needed.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        // Default: no initialization needed
    }

protected:
    /**
     * @brief Common compute logic for IMU publishing nodes.
     * @details
     * Handles listener initialization, connection checking, and message publishing with timeout.
     * Delegates to publishMessage() for actual message generation and sending.
     * Sets execOut port on success.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @param[in] port Port number for UCX listener
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds for send request (0 = infinite)
     * @return bool True if execution succeeded, false otherwise
     */
    bool computeImpl(DatabaseT& db, uint16_t port, uint64_t tag, uint32_t timeoutMs)
    {
        if (!this->ensureListenerReady(db, port))
        {
            return false;
        }

        if (!this->waitForConnection())
        {
            return true;
        }

        bool success = publishMessage(db, tag, timeoutMs);

        if (success)
        {
            db.outputs.execOut() = kExecutionAttributeStateEnabled;
        }

        return success;
    }

    /**
     * @brief Generate message from node inputs.
     * @details
     * Pure virtual function that derived classes must implement to create
     * and serialize their IMU message data.
     *
     * @param[in] db Database accessor for node inputs
     * @return std::vector<uint8_t> Serialized message data
     */
    virtual std::vector<uint8_t> generateMessage(DatabaseT& db) = 0;

    /**
     * @brief Publishes an IMU message over UCX with timeout.
     * @details
     * Generates the message by calling the derived class's virtual generateMessage(),
     * then sends it using UCX tagged send and waits for completion with timeout.
     *
     * @param[in] db Database accessor for logging and inputs
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds for send request (0 = infinite)
     * @return bool True if publish succeeded, false otherwise
     */
    bool publishMessage(DatabaseT& db, uint64_t tag, uint32_t timeoutMs)
    {
        std::vector<uint8_t> messageData = generateMessage(db);

        if (messageData.empty())
        {
            db.logError("Failed to generate message");
            return false;
        }

        return this->sendMessage(db, messageData, tag, timeoutMs);
    }
};
