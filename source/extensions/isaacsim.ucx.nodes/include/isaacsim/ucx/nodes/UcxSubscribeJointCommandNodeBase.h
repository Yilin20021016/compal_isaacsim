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

#include <cstring>
#include <vector>

namespace isaacsim
{
namespace ucx
{
namespace nodes
{


/**
 * @brief Base class template for UCX joint state subscriber nodes.
 * @details
 * Provides common functionality for receiving joint command data via UCX.
 *
 * @tparam DatabaseT The OmniGraph database type for the derived node
 */
template <typename DatabaseT>
class UCXSubscribeJointCommandNodeBase : public UcxNode
{
public:
    using NodeObj = omni::graph::core::NodeObj;
    using GraphInstanceID = omni::graph::core::GraphInstanceID;

    /**
     * @brief Initialize the node instance.
     * @details
     * Sets up the node state and initializes member variables.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state = DatabaseT::template sPerInstanceState<UCXSubscribeJointCommandNodeBase>(nodeObj, instanceId);
        state.m_receiveBuffer.reserve(65536); // Reserve 64KB for receive buffer
    }

    /**
     * @brief Reset the node state.
     * @details
     * Clears output arrays and resets the connection.
     */
    virtual void reset() override
    {
        m_receiveBuffer.clear();
        UcxNode::reset();
    }

protected:
    /**
     * @brief Compute implementation for joint state subscription.
     * @details
     * Receives joint command data via UCX and outputs it.
     *
     * @param[in,out] db Database accessor for node inputs/outputs
     * @param[in] port Port number for UCX communication
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds for receive request (0 = infinite)
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

        return receiveMessage(db, tag, timeoutMs);
    }

    /**
     * @brief Receive and deserialize joint command message.
     * @details
     * Receives a message via UCX and deserializes it into joint command outputs.
     * If the request doesn't complete within the timeout, it is cancelled.
     * Message format: timestamp(8) + num_dofs(4) + positions(double*num_dofs) +
     *                 velocities(double*num_dofs) + efforts(double*num_dofs)
     *
     * @param[in,out] db Database accessor for node inputs/outputs
     * @param[in] tag UCX tag for message identification
     * @param[in] timeoutMs Timeout in milliseconds (0 = infinite)
     * @return bool True if receive succeeded, false otherwise
     */
    virtual bool receiveMessage(DatabaseT& db, uint64_t tag, uint32_t timeoutMs)
    {
        try
        {
            const size_t maxBufferSize = 65536;
            m_receiveBuffer.resize(maxBufferSize);

            std::string errorMessage;
            auto result = this->m_listener->tagReceive(
                m_receiveBuffer.data(), m_receiveBuffer.size(), tag, 0xFFFFFFFFFFFFFFFF, errorMessage, timeoutMs);

            if (result == isaacsim::ucx::core::UcxReceiveResult::eTimedOut)
            {
                db.logWarning("Receive request timed out after %u ms: %s", timeoutMs, errorMessage.c_str());
                return true;
            }
            else if (result == isaacsim::ucx::core::UcxReceiveResult::eFailed)
            {
                db.logWarning("Receive request failed: %s", errorMessage.c_str());
                return true;
            }
            else if (result != isaacsim::ucx::core::UcxReceiveResult::eSuccess)
            {
                db.logWarning("Receive request returned unexpected result: %s", errorMessage.c_str());
                return true;
            }

            size_t offset = 0;

            double timestamp;
            std::memcpy(&timestamp, m_receiveBuffer.data() + offset, sizeof(double));
            offset += sizeof(double);

            uint32_t numDofs;
            std::memcpy(&numDofs, m_receiveBuffer.data() + offset, sizeof(uint32_t));
            offset += sizeof(uint32_t);

            db.outputs.positionCommand().resize(numDofs);
            db.outputs.velocityCommand().resize(numDofs);
            db.outputs.effortCommand().resize(numDofs);

            std::memcpy(db.outputs.positionCommand().data(), m_receiveBuffer.data() + offset, sizeof(double) * numDofs);
            offset += sizeof(double) * numDofs;

            std::memcpy(db.outputs.velocityCommand().data(), m_receiveBuffer.data() + offset, sizeof(double) * numDofs);
            offset += sizeof(double) * numDofs;

            std::memcpy(db.outputs.effortCommand().data(), m_receiveBuffer.data() + offset, sizeof(double) * numDofs);
            offset += sizeof(double) * numDofs;

            db.outputs.timeStamp() = timestamp;
            db.outputs.execOut() = kExecutionAttributeStateEnabled;

            return true;
        }
        catch (const std::exception& e)
        {
            db.logError("Exception during message receive: %s", e.what());
            return false;
        }
    }

    std::vector<uint8_t> m_receiveBuffer;
};

} // namespace nodes
} // namespace ucx
} // namespace isaacsim
