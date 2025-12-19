# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: Apache-2.0
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import struct
import time

import numpy as np
import omni.graph.core as og
import omni.kit.commands
import omni.kit.test
import omni.kit.usd
import ucxx._lib.libucxx as ucx_api
import usdrt.Sdf
from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.physics import simulate_async
from isaacsim.core.utils.stage import open_stage_async
from isaacsim.storage.native import get_assets_root_path
from ucxx._lib.arr import Array

from .common import UCXTestCase


def unpack_joint_state_message(buffer):
    """Unpack a UCX joint state message.

    Message format (updated to use doubles):
    - timestamp (double, 8 bytes)
    - num_joints (uint32_t, 4 bytes)
    - For each joint:
      - position (double, 8 bytes)
      - velocity (double, 8 bytes)
      - effort (double, 8 bytes)

    Args:
        buffer: Buffer containing the packed joint state message.

    Returns:
        Tuple of (timestamp, num_joints, positions, velocities, efforts).
    """
    offset = 0
    timestamp = struct.unpack("<d", buffer[offset : offset + 8].tobytes())[0]
    offset += 8

    num_joints = struct.unpack("<I", buffer[offset : offset + 4].tobytes())[0]
    offset += 4

    positions = []
    velocities = []
    efforts = []

    for i in range(num_joints):
        position = struct.unpack("<d", buffer[offset : offset + 8].tobytes())[0]
        offset += 8
        positions.append(position)

        velocity = struct.unpack("<d", buffer[offset : offset + 8].tobytes())[0]
        offset += 8
        velocities.append(velocity)

        effort = struct.unpack("<d", buffer[offset : offset + 8].tobytes())[0]
        offset += 8
        efforts.append(effort)

    return timestamp, num_joints, positions, velocities, efforts


class TestUCXPublishJointState(UCXTestCase):
    """Test UCX joint state publishing"""

    async def setUp(self):
        await super().setUp()
        await omni.usd.get_context().new_stage_async()
        await omni.kit.app.get_app().next_update_async()

        # Load simple articulation asset
        assets_root_path = get_assets_root_path()
        if assets_root_path is None:
            raise RuntimeError("Could not find Isaac Sim assets folder")

        self.usd_path = assets_root_path + "/Isaac/Robots/IsaacSim/SimpleArticulation/articulation_3_joints.usd"
        (result, error) = await open_stage_async(self.usd_path)
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(result)
        self._stage = omni.usd.get_context().get_stage()

        # Create UCX publish joint state node
        try:
            og.Controller.edit(
                {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
                {
                    og.Controller.Keys.CREATE_NODES: [
                        ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                        ("PublishJointState", "isaacsim.ucx.nodes.UCXPublishJointState"),
                        ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                    ],
                    og.Controller.Keys.SET_VALUES: [
                        ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path("/Articulation")]),
                        ("PublishJointState.inputs:port", self.port),
                        ("PublishJointState.inputs:tag", 1),
                        ("PublishJointState.inputs:timeoutMs", 1000),
                    ],
                    og.Controller.Keys.CONNECT: [
                        ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                        ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                    ],
                },
            )
        except Exception as e:
            print(f"Error creating graph: {e}")
            raise

        # Start timeline FIRST so the node executes and creates its listener
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()

        for _ in range(3):
            await omni.kit.app.get_app().next_update_async()
        self.create_ucx_client(self.port)

    async def receive_joint_state_message(self, tag=1, timeout_frames=1000):
        """Receive and unpack a joint state message from the client endpoint"""
        max_buffer_size = 512
        buffer = np.empty(max_buffer_size, dtype=np.uint8)

        # Receive using the endpoint
        request = self.client_endpoint.tag_recv(Array(buffer), tag=ucx_api.UCXXTag(tag))

        # Progress until complete

        for _ in range(timeout_frames):
            if request.completed:
                break
            time.sleep(0.001)
            await omni.kit.app.get_app().next_update_async()

        # Check if completed
        self.assertTrue(request.completed, "Did not receive joint state message")
        request.check_error()

        return unpack_joint_state_message(buffer)

    async def test_joint_state_publisher(self):
        """Test that joint state messages are published via UCX"""
        # Receive joint state message
        timestamp, num_joints, positions, velocities, efforts = await self.receive_joint_state_message()

        print(f"Received joint state: {num_joints} joints")
        print(f"  Timestamp: {timestamp}")
        print(f"  Positions: {positions}")
        print(f"  Velocities: {velocities}")
        print(f"  Efforts: {efforts}")

        # Verify we got data
        self.assertGreater(num_joints, 0, "Should have at least one joint")
        self.assertEqual(len(positions), num_joints)
        self.assertEqual(len(velocities), num_joints)
        self.assertEqual(len(efforts), num_joints)
        self.assertGreater(timestamp, 0.0, "Timestamp should be positive")

    async def test_joint_state_values(self):
        """Test that joint state values match simulation"""
        # Simulate to initialize physics
        await simulate_async(0.5)

        # Get articulation and initialize it
        articulation = SingleArticulation(prim_path="/Articulation")
        articulation.initialize()

        # Simulate for a few frames to let physics settle
        await simulate_async(0.5)

        # Get joint states from simulation
        sim_positions = articulation.get_joint_positions()
        sim_velocities = articulation.get_joint_velocities()

        # Receive joint state message from UCX
        timestamp, num_joints, ucx_positions, ucx_velocities, ucx_efforts = await self.receive_joint_state_message()

        # Verify values match (within tolerance)
        self.assertEqual(num_joints, len(sim_positions))

        for i in range(num_joints):
            self.assertAlmostEqual(ucx_positions[i], sim_positions[i], places=3, msg=f"Joint {i} position mismatch")
            self.assertAlmostEqual(ucx_velocities[i], sim_velocities[i], places=3, msg=f"Joint {i} velocity mismatch")
