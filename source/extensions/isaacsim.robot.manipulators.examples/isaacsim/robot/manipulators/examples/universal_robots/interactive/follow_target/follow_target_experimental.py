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

"""
UR10 Follow Target Interactive Example

This interactive example demonstrates the UR10 robot following a target cube
without complex layers or RL concepts. Users can move the target cube and watch
the robot follow it through the UI.
"""

import numpy as np
import omni.kit.app
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.examples.base.base_sample_experimental import BaseSample
from isaacsim.robot.manipulators.examples.universal_robots import UR10FollowTarget


class UR10FollowTargetInteractive(BaseSample):
    """Interactive sample for UR10 follow target demonstration."""

    def __init__(self) -> None:
        super().__init__()
        self.controller: UR10FollowTarget = None
        self._is_following = False
        self._ik_method = "damped-least-squares"
        self._physics_callback_id = None

    def setup_scene(self):
        """Set up the scene with UR10 robot and target cube."""
        # Create controller and setup scene
        self.controller = UR10FollowTarget()
        self.controller.setup_scene(target_position=[0.4, 0.2, 0.3])

        print("UR10 follow target scene setup complete")

    async def setup_post_load(self):
        """Called after the scene is loaded."""
        print("UR10 follow target scene loaded successfully")

    async def setup_pre_reset(self):
        """Called before world reset."""
        # Stop any ongoing following and remove callbacks
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                # Callback may have already been deregistered or doesn't exist
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        if self.controller:
            self.controller.reset_robot()
        self._is_following = False

    async def setup_post_reset(self):
        """Called after world reset."""
        if self.controller:
            # Ensure robot starts in a good position
            self.controller.reset_robot()

    async def setup_post_clear(self):
        """Called after clearing the scene."""
        # Stop any ongoing following and remove callbacks
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                # Callback may have already been deregistered or doesn't exist
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        self.controller = None
        self._is_following = False

    def physics_cleanup(self):
        """Clean up world resources."""
        # Stop any ongoing following and remove callbacks
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                # Callback may have already been deregistered or doesn't exist
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        self.controller = None
        self._is_following = False

    def _follow_target_physics_callback(self, dt, context):
        """Physics callback to execute follow target step by step."""
        if not self._is_following or self.controller is None:
            return

        # Execute one step of the follow target operation
        try:
            self.controller.move_to_target(ik_method=self._ik_method)
        except Exception as e:
            print(f"Error during follow target step: {e}")
            self._is_following = False
            if self._physics_callback_id is not None:
                try:
                    SimulationManager.deregister_callback(self._physics_callback_id)
                except Exception as deregister_error:
                    print(f"Note: Could not deregister callback {self._physics_callback_id}: {deregister_error}")
                self._physics_callback_id = None

    def get_controller_status(self) -> dict:
        """Get current status of the controller."""
        if self.controller:
            target_pos = self.controller.get_target_position()
            ee_pos = self.controller.get_robot_end_effector_position()
            distance = np.linalg.norm(target_pos - ee_pos)

            return {
                "target_position": target_pos.tolist(),
                "end_effector_position": ee_pos.tolist(),
                "distance_to_target": float(distance),
                "target_reached": self.controller.target_reached(),
                "is_following": self._is_following,
                "ik_method": self._ik_method,
            }
        else:
            return {"error": "Controller not initialized"}

    def is_following(self) -> bool:
        """Check if the robot is currently following the target."""
        return self._is_following

    def set_ik_method(self, method: str):
        """Set the inverse kinematics method to use."""
        valid_methods = ["damped-least-squares", "pseudoinverse", "transpose", "singular-value-decomposition"]
        if method in valid_methods:
            self._ik_method = method
            print(f"IK method set to: {method}")
        else:
            print(f"Invalid IK method: {method}. Valid methods: {valid_methods}")

    # Methods for UI integration
    async def start_following_async(self):
        """Start the follow target behavior."""
        if self.controller is None:
            print("ERROR: Controller not initialized")
            return False

        if self._is_following:
            print("Already following target...")
            return False

        print("Starting follow target execution...")
        self._is_following = True

        # Reset the robot to ensure clean start
        self.controller.reset_robot()
        print("Robot reset for clean start")

        # Register physics callback using SimulationManager
        from isaacsim.core.simulation_manager.impl.isaac_events import IsaacEvents

        self._physics_callback_id = SimulationManager.register_callback(
            self._follow_target_physics_callback, IsaacEvents.POST_PHYSICS_STEP
        )

        # Start timeline playback
        self._timeline.play()
        await omni.kit.app.get_app().next_update_async()
        return True

    async def stop_following_async(self):
        """Stop the follow target behavior."""
        if not self._is_following:
            print("Not currently following target...")
            return False

        print("Stopping follow target execution...")
        self._is_following = False

        # Remove the physics callback
        if self._physics_callback_id is not None:
            try:
                SimulationManager.deregister_callback(self._physics_callback_id)
            except Exception as e:
                # Callback may have already been deregistered or doesn't exist
                print(f"Note: Could not deregister callback {self._physics_callback_id}: {e}")
            self._physics_callback_id = None

        print("Follow target execution stopped")
        return True
