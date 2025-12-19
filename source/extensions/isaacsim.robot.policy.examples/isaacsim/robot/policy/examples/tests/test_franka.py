# SPDX-FileCopyrightText: Copyright (c) 2018-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import asyncio

import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils

# NOTE:
#   omni.kit.test - std python's unittest module with additional wrapping to add suport for async/await tests
#   For most things refer to unittest docs: https://docs.python.org/3/library/unittest.html
import omni.kit.test
import omni.timeline
from isaacsim.core.deprecation_manager import import_module
from isaacsim.core.experimental.prims import Articulation
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents
from isaacsim.robot.policy.examples.robots.franka import FrankaOpenDrawerPolicy
from isaacsim.storage.native import get_assets_root_path
from pxr import UsdPhysics

torch = import_module("torch")


class TestFrankaExampleExtension(omni.kit.test.AsyncTestCase):
    def get_device(self):
        """Return the device to use for tensors. Override in subclasses."""
        return torch.device("cpu")

    async def setUp(self):
        await stage_utils.create_new_stage_async()
        # This needs to be set so that kit updates match physics updates
        self._physics_rate = 400

        device_str = str(self.get_device())
        backend = "torch" if device_str != "cpu" else "numpy"

        print(f"Setting up test with device: {device_str}, backend: {backend}")

        self._physics_dt = 1 / self._physics_rate
        stage_utils.define_prim("/World/PhysicsScene", "PhysicsScene")

        # spawn simulation manager
        SimulationManager.set_physics_sim_device(device_str)
        SimulationManager.set_physics_dt(self._physics_dt)

        ground_plane = stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

        cabinet_prim_path = "/World/cabinet"
        cabinet_usd_path = get_assets_root_path() + "/Isaac/Props/Sektion_Cabinet/sektion_cabinet_instanceable.usd"

        cabinet_position = [0.8, 0.0, 0.4]
        cabinet_orientation = [0.0, 0.0, 0.0, 1.0]

        stage_utils.add_reference_to_stage(cabinet_usd_path, cabinet_prim_path)

        self.cabinet = Articulation(cabinet_prim_path, positions=cabinet_position, orientations=cabinet_orientation)

        self._timeline = omni.timeline.get_timeline_interface()
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        self._timeline.stop()
        SimulationManager.deregister_callback(self._physics_callback_id)

        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await omni.kit.app.get_app().next_update_async()

    async def test_franka_add(self):
        await self.spawn_franka()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()
        self.assertEqual(self._franka.robot.num_dofs, 9)

        # Verify robot prim exists
        robot_prim = stage_utils.get_current_stage().GetPrimAtPath("/World/franka")
        self.assertIsNotNone(robot_prim, "Robot prim should exist in stage at /World/franka")
        self.assertTrue(robot_prim.IsValid(), "Robot prim should be valid")
        self.assertTrue(
            prim_utils.has_api(robot_prim, UsdPhysics.ArticulationRootAPI),
            "Robot base prim should have ArticulationRootAPI",
        )

        # Verify cabinet prim exists
        cabinet_prim = stage_utils.get_current_stage().GetPrimAtPath("/World/cabinet")
        self.assertIsNotNone(cabinet_prim, "Cabinet prim should exist in stage at /World/cabinet")
        self.assertTrue(cabinet_prim.IsValid(), "Cabinet prim should be valid")

    async def test_franka_open_drawer(self):
        await self.spawn_franka()
        await omni.kit.app.get_app().next_update_async()
        max_drawer_opening = 0
        drawer_link_idx = self.cabinet.get_dof_indices("drawer_top_joint")
        for i in range(200):
            # Step the simulation
            await omni.kit.app.get_app().next_update_async()
            drawer_joint_position = self.cabinet.get_dof_positions(indices=[0], dof_indices=drawer_link_idx)
            # Convert Warp array to numpy array and extract the scalar value
            drawer_position_numpy = drawer_joint_position.numpy()
            drawer_position_scalar = float(drawer_position_numpy[0][0])
            if drawer_position_scalar > max_drawer_opening:
                max_drawer_opening = drawer_position_scalar

        # drawer is closed at 0.0 and open at 0.4, the robot should be able to open the drawer
        # to at least 0.3
        self.assertGreater(max_drawer_opening, 0.3)

    async def spawn_franka(self, name="franka", add_physics_callback=True):
        self._prim_path = "/World/" + name

        self._franka = FrankaOpenDrawerPolicy(prim_path=self._prim_path, position=[0, 0, 0], cabinet=self.cabinet)
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self._franka.initialize()
        self._franka.post_reset()
        self._franka.robot.set_dof_positions(self._franka.default_pos.cpu().numpy().tolist())
        await omni.kit.app.get_app().next_update_async()

        if add_physics_callback:
            self._physics_callback_id = SimulationManager.register_callback(
                self.on_physics_step, IsaacEvents.POST_PHYSICS_STEP
            )
        await omni.kit.app.get_app().next_update_async()

    def on_physics_step(self, step_size, context):
        if self._franka:
            self._franka.forward(step_size)


class TestFrankaGPU(TestFrankaExampleExtension):
    def get_device(self):
        """Return the device to use for tensors"""
        return torch.device("cuda")
