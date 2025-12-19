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

// clang-format off
#include <pch/UsdPCH.h>
// clang-format on

#include <isaacsim/ucx/nodes/UcxSubscribeJointCommandNodeBase.h>

#include <OgnUCXSubscribeJointCommandDatabase.h>

using namespace isaacsim::ucx::nodes;

/**
 * @class OgnUCXSubscribeJointCommand
 * @brief OmniGraph node for subscribing to joint state commands via UCX.
 * @details
 * This node receives joint command data over UCX using tagged communication.
 * It outputs position, velocity, and effort commands for robot joints.
 */
class OgnUCXSubscribeJointCommand : public UCXSubscribeJointCommandNodeBase<OgnUCXSubscribeJointCommandDatabase>
{
public:
    /**
     * @brief Initialize the node instance.
     * @details
     * Sets up the node state when the node is first created.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void initInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        UCXSubscribeJointCommandNodeBase<OgnUCXSubscribeJointCommandDatabase>::initInstance(nodeObj, instanceId);
    }

    /**
     * @brief Release the node instance.
     * @details
     * Cleans up resources when the node instance is destroyed.
     *
     * @param[in] nodeObj The node object
     * @param[in] instanceId The instance ID
     */
    static void releaseInstance(NodeObj const& nodeObj, GraphInstanceID instanceId)
    {
        auto& state =
            OgnUCXSubscribeJointCommandDatabase::sPerInstanceState<OgnUCXSubscribeJointCommand>(nodeObj, instanceId);
        state.reset();
    }

    /**
     * @brief Compute function - called when node is executed.
     * @details
     * Receives joint command data via UCX and outputs it.
     *
     * @param[in] db Database accessor for node inputs/outputs
     * @return bool True if execution succeeded, false otherwise
     */
    static bool compute(OgnUCXSubscribeJointCommandDatabase& db)
    {
        auto& state = db.template perInstanceState<OgnUCXSubscribeJointCommand>();

        const uint16_t port = static_cast<uint16_t>(db.inputs.port());
        const uint64_t tag = db.inputs.tag();
        const uint32_t timeoutMs = db.inputs.timeoutMs();

        return state.computeImpl(db, port, tag, timeoutMs);
    }
};

REGISTER_OGN_NODE()
