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
import typing

import numpy as np
from isaacsim.core.api.controllers.base_controller import BaseController
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.controllers.pick_place_controller import PickPlaceController


class StackingController(BaseController):
    """Controller for stacking objects in a specified order.

    Args:
        name: Name identifier for the controller.
        pick_place_controller: The underlying pick and place controller.
        picking_order_cube_names: Ordered list of cube names to pick and stack.
        robot_observation_name: Name key for robot observations in the observation dict.
    """

    def __init__(
        self,
        name: str,
        pick_place_controller: PickPlaceController,
        picking_order_cube_names: typing.List[str],
        robot_observation_name: str,
    ) -> None:
        BaseController.__init__(self, name=name)
        self._pick_place_controller = pick_place_controller
        self._picking_order_cube_names = picking_order_cube_names
        self._current_cube = 0
        self._robot_observation_name = robot_observation_name
        self.reset()

    def forward(
        self,
        observations: dict,
        end_effector_orientation: typing.Optional[np.ndarray] = None,
        end_effector_offset: typing.Optional[np.ndarray] = None,
    ) -> ArticulationAction:
        if self._current_cube >= len(self._picking_order_cube_names):
            target_joint_positions = [None] * observations[self._robot_observation_name]["joint_positions"].shape[0]
            return ArticulationAction(joint_positions=target_joint_positions)
        actions = self._pick_place_controller.forward(
            picking_position=observations[self._picking_order_cube_names[self._current_cube]]["position"],
            placing_position=observations[self._picking_order_cube_names[self._current_cube]]["target_position"],
            current_joint_positions=observations[self._robot_observation_name]["joint_positions"],
            end_effector_orientation=end_effector_orientation,
            end_effector_offset=end_effector_offset,
        )
        if self._pick_place_controller.is_done():
            self._current_cube += 1
            self._pick_place_controller.reset()
        return actions

    def reset(self, picking_order_cube_names: typing.Optional[typing.List[str]] = None) -> None:
        """Reset the controller state and optionally update the picking order.

        Args:
            picking_order_cube_names: New list of cube names to pick in order. Defaults to None.
        """
        self._current_cube = 0
        self._pick_place_controller.reset()
        if picking_order_cube_names is not None:
            self._picking_order_cube_names = picking_order_cube_names
        return

    def is_done(self) -> bool:
        """Check if all cubes have been stacked.

        Returns:
            True if all cubes have been stacked, False otherwise.
        """
        if self._current_cube >= len(self._picking_order_cube_names):
            return True
        else:
            return False
