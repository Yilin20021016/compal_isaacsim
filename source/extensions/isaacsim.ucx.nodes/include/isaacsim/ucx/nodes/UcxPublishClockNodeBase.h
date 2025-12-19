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
#include <isaacsim/ucx/nodes/UcxNode.h>
#include <omni/graph/core/CppWrappers.h>
#include <omni/graph/core/iComputeGraph.h>

#include <cstring>
#include <vector>


using omni::graph::core::GraphInstanceID;
using omni::graph::core::NodeObj;

/**
 * @class UCXPublishClockNodeBase
 * @brief Templated base class for UCX clock publishing nodes.
 * @details
 * This template provides common functionality for publishing data over UCX.
 * Derived classes implement message generation logic via generateMessage().
 *
 * @tparam DatabaseT The OGN database type for the node
 */
template <typename DatabaseT>
class UCXPublishClockNodeBase : public isaacsim::ucx::nodes::UcxNode
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
     * @brief Common compute logic for clock publishing nodes.
     * @details
     * Handles listener initialization, connection checking, and message publishing with timeout.
     * Delegates to publishMessage() for actual message generation and sending.
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

        return publishMessage(db, tag, timeoutMs);
    }

    /**
     * @brief Publishes a message over UCX with timeout.
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

    /**
     * @brief Generate message from node inputs.
     * @details
     * Pure virtual function that derived classes must implement to create
     * and serialize their message data.
     *
     * @param[in] db Database accessor for node inputs
     * @return std::vector<uint8_t> Serialized message data
     */
    virtual std::vector<uint8_t> generateMessage(DatabaseT& db) = 0;
};

// NOTE: To use this base class:
// 1. Derive your OGN node class from UCXPublishClockNodeBase<YourDatabase>
// 2. Implement static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
//    - Get state: auto& state = YourDatabase::template sPerInstanceState<YourClass>(nodeObj, instanceId)
//    - Call state.reset()
// 3. Implement virtual std::vector<uint8_t> generateMessage(DatabaseT& db) override
//    This protected function should read inputs from db and return the serialized message bytes
// 4. Implement static bool compute(YourDatabase& db) that:
//    - Extracts inputs from db
//    - Gets the per-instance state: auto& state = db.template perInstanceState<YourClass>()
//    - Calls state.computeImpl(db, port, tag)
// 5. See OgnUCXPublishClock.cpp and OgnUCXPublishClockOffset.cpp for examples
