# SPDX-FileCopyrightText: Copyright (c) 2021-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import time

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.timeline
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager
from isaacsim.core.utils.viewports import set_camera_view
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.storage.native import get_assets_root_path


class GettingStartedRobot(BaseSample):
    def __init__(self) -> None:
        super().__init__()
        self._timeline = omni.timeline.get_timeline_interface()
        self.print_state = False
        self.car_handle = None
        self.arm_handle = None
        self._physics_callback_id = None

    def setup_scene(self):
        """Set up the scene with ground plane using experimental API."""
        # Add default environment using experimental stage utils
        stage_utils.add_reference_to_stage(
            usd_path=get_assets_root_path() + "/Isaac/Environments/Grid/default_environment.usd",
            path="/World/ground",
        )

    async def setup_post_load(self):
        # move camera to a better vanatage point
        set_camera_view(eye=[5.0, 0.0, 1.5], target=[0.00, 0.00, 1.00], camera_prim_path="/OmniverseKit_Persp")

        # Add physics callback using SimulationManager (experimental API)
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, event=IsaacEvents.POST_PHYSICS_STEP
        )

        # do a quick start and stop to reset the physics timeline
        self._timeline.play()
        time.sleep(1)
        self._timeline.stop()

    def on_physics_step(self, step_size, context) -> None:
        """Physics callback - note the signature includes context parameter."""
        if self.print_state:
            if self.arm_handle:
                print("arm joint state: ", self.arm_handle.get_dof_positions())
            if self.car_handle:
                print("car joint state: ", self.car_handle.get_dof_positions())

    async def setup_pre_reset(self):
        # Remove physics callback before reset
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                print(f"Note: Could not deregister callback: {e}")
            self._physics_callback_id = None

    async def setup_post_reset(self):
        # Re-register physics callback after reset
        self._physics_callback_id = SimulationManager.register_callback(
            self.on_physics_step, event=IsaacEvents.POST_PHYSICS_STEP
        )
        self._timeline.stop()

    async def setup_post_clear(self):
        """Called after clearing the scene."""
        # Remove physics callback on clear
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                print(f"Note: Could not deregister callback: {e}")
            self._physics_callback_id = None

    def physics_cleanup(self):
        """Clean up physics resources."""
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                print(f"Note: Could not deregister callback: {e}")
            self._physics_callback_id = None
