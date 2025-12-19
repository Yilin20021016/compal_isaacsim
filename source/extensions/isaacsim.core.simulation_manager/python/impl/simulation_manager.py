# SPDX-FileCopyrightText: Copyright (c) 2024-2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
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

import weakref
from collections import OrderedDict
from typing import Callable

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.experimental.utils.prim as prim_utils
import isaacsim.core.experimental.utils.stage as stage_utils
import omni.physics.core
import omni.physx
import omni.timeline
import omni.usd
from pxr import PhysxSchema

from .isaac_events import IsaacEvents
from .simulation_event import SimulationEvent

_SETTING_PLAY_SIMULATION = "/app/player/playSimulations"


class SimulationManager:
    """This class provide functions that take care of many time-related events such as
    warm starting simulation in order for the physics data to be retrievable.
    Adding/ removing callback functions that gets triggered with certain events such as a physics step,
    on post reset, on physics ready..etc."""

    _warmup_needed = True
    _timeline = omni.timeline.get_timeline_interface()
    _message_bus = carb.eventdispatcher.get_eventdispatcher()
    _physics_sim_interface = omni.physics.core.get_physics_simulation_interface()
    _physics_stage_update_interface = omni.physics.core.get_physics_stage_update_interface()
    _physx_sim_interface = omni.physx.get_physx_simulation_interface()
    _physx_interface = omni.physx.get_physx_interface()
    _physx_fabric_interface = None
    _physics_sim_view = None
    _physics_sim_view__warp = None
    _backend = "numpy"
    _carb_settings = carb.settings.get_settings()
    _physics_scene_apis = OrderedDict()
    _callbacks = dict()
    _simulation_manager_interface = None
    _simulation_view_created = False
    _assets_loaded = True
    _assets_loading_callback = None
    _assets_loaded_callback = None
    _default_physics_scene_idx = -1

    # callback handles
    _warm_start_callback = None
    _on_stop_callback = None
    _stage_open_callback = None

    # Add callback state tracking
    _callbacks_enabled = {
        "warm_start": True,
        "on_stop": True,
        "post_warm_start": True,
        "stage_open": True,
    }

    """
    Internal methods.
    """

    @classmethod
    def _initialize(cls) -> None:
        # Initialize all callbacks as enabled by default
        SimulationManager.enable_all_default_callbacks(True)
        SimulationManager._track_physics_scenes()

    @classmethod
    def _clear(cls) -> None:
        # Use callback management system for main callbacks
        cls.enable_all_default_callbacks(False)

        # Handle additional cleanup not covered by enable_all_default_callbacks
        cls._physics_sim_view = None
        cls._physics_sim_view__warp = None
        cls._assets_loading_callback = None
        cls._assets_loaded_callback = None
        cls._simulation_manager_interface.reset()
        cls._physics_scene_apis.clear()
        cls._callbacks.clear()

    @classmethod
    def _setup_warm_start_callback(cls) -> None:
        if cls._callbacks_enabled["warm_start"] and cls._warm_start_callback is None:
            cls._warm_start_callback = cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.PLAY), cls._on_play
            )

    @classmethod
    def _setup_on_stop_callback(cls) -> None:
        if cls._callbacks_enabled["on_stop"] and cls._on_stop_callback is None:
            cls._on_stop_callback = cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                int(omni.timeline.TimelineEventType.STOP), cls._on_stop
            )

    @classmethod
    def _setup_stage_open_callback(cls) -> None:
        if cls._callbacks_enabled["stage_open"] and cls._stage_open_callback is None:
            cls._stage_open_callback = cls._message_bus.observe_event(
                event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.OPENED),
                on_event=cls._on_stage_opened,
                observer_name="SimulationManager._stage_open_callback",
            )

    def _track_physics_scenes() -> None:
        def add_physics_scenes(physics_scene_prim_path):
            prim = prim_utils.get_prim_at_path(physics_scene_prim_path)
            if prim.GetTypeName() == "PhysicsScene":
                SimulationManager._physics_scene_apis[physics_scene_prim_path] = PhysxSchema.PhysxSceneAPI.Apply(prim)

        def remove_physics_scenes(physics_scene_prim_path):
            # TODO: match physics scene prim path
            if physics_scene_prim_path in SimulationManager._physics_scene_apis:
                del SimulationManager._physics_scene_apis[physics_scene_prim_path]

        SimulationManager._simulation_manager_interface.register_physics_scene_addition_callback(add_physics_scenes)
        SimulationManager._simulation_manager_interface.register_deletion_callback(remove_physics_scenes)

    @classmethod
    def _get_backend_utils(cls) -> str:
        # defer imports to avoid an explicit dependency on the deprecated core API
        if SimulationManager._backend == "numpy":
            import isaacsim.core.utils.numpy as np_utils

            return np_utils
        elif SimulationManager._backend == "torch":
            import isaacsim.core.utils.torch as torch_utils

            return torch_utils
        elif SimulationManager._backend == "warp":
            import isaacsim.core.utils.warp as warp_utils

            return warp_utils
        else:
            raise Exception(
                f"Provided backend is not supported: {SimulationManager.get_backend()}. Supported: torch, numpy, warp."
            )

    @classmethod
    def _get_physics_scene_api(cls, physics_scene: str = None):
        if physics_scene is None:
            if len(SimulationManager._physics_scene_apis) > 0:
                physics_scene_api = list(SimulationManager._physics_scene_apis.values())[
                    SimulationManager._default_physics_scene_idx
                ]
            else:
                # carb.log_warn("Physics scene is not found in stage")
                return None
        else:
            if physics_scene in SimulationManager._physics_scene_apis:
                physics_scene_api = SimulationManager._physics_scene_apis[physics_scene]
            else:
                carb.log_warn("physics scene specified {} doesn't exist".format(physics_scene))
                return None
        return physics_scene_api

    """
    Internal callbacks.
    """

    def _on_stage_opened(event) -> None:
        SimulationManager._simulation_manager_interface.reset()
        SimulationManager._physics_scene_apis.clear()
        SimulationManager._callbacks.clear()
        SimulationManager._track_physics_scenes()
        SimulationManager._assets_loaded = True
        SimulationManager._assets_loading_callback = None
        SimulationManager._assets_loaded_callback = None

        def _assets_loading(event):
            SimulationManager._assets_loaded = False

        def _assets_loaded(event):
            SimulationManager._assets_loaded = True

        SimulationManager._assets_loading_callback = SimulationManager._message_bus.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.ASSETS_LOADING),
            on_event=_assets_loading,
            observer_name="SimulationManager._assets_loading_callback",
        )

        SimulationManager._assets_loaded_callback = SimulationManager._message_bus.observe_event(
            event_name=omni.usd.get_context().stage_event_name(omni.usd.StageEventType.ASSETS_LOADED),
            on_event=_assets_loaded,
            observer_name="SimulationManager._assets_loaded_callback",
        )

    def _on_play(event) -> None:
        if SimulationManager._carb_settings.get_as_bool(_SETTING_PLAY_SIMULATION):
            if SimulationManager._warmup_needed:
                SimulationManager.initialize_physics()
                SimulationManager._message_bus.dispatch_event(SimulationEvent.SIMULATION_STARTED.value, payload={})
            else:
                SimulationManager._message_bus.dispatch_event(SimulationEvent.SIMULATION_RESUMED.value, payload={})

    def _on_stop(event) -> None:
        SimulationManager._warmup_needed = True
        if SimulationManager._physics_sim_view:
            SimulationManager._physics_sim_view.invalidate()
            SimulationManager._physics_sim_view = None
            SimulationManager._simulation_view_created = False
        if SimulationManager._physics_sim_view__warp:
            SimulationManager._physics_sim_view__warp.invalidate()
            SimulationManager._physics_sim_view__warp = None

    """
    Public methods.
    """

    @classmethod
    def set_backend(cls, val: str) -> None:
        """Set the backend used by the simulation manager.

        .. deprecated:: 1.7.0

            |br| No replacement is provided, as the core experimental API relies solely on Warp.
        """
        SimulationManager._backend = val

    @classmethod
    def get_backend(cls) -> str:
        """Get the backend used by the simulation manager.

        .. deprecated:: 1.7.0

            |br| No replacement is provided, as the core experimental API relies solely on Warp.
        """
        return SimulationManager._backend

    @classmethod
    def initialize_physics(cls) -> None:
        if not SimulationManager._warmup_needed:
            return
        # initialize physics engine
        SimulationManager._physics_stage_update_interface.force_load_physics_from_usd()
        SimulationManager._physx_interface.start_simulation()
        SimulationManager._physics_sim_interface.simulate(SimulationManager.get_physics_dt(), 0.0)
        SimulationManager._physics_sim_interface.fetch_results()
        # create simulation view
        stage_id = stage_utils.get_stage_id(stage_utils.get_current_stage(backend="usd"))
        SimulationManager._physics_sim_view__warp = omni.physics.tensors.create_simulation_view(
            "warp", stage_id=stage_id
        )
        SimulationManager._physics_sim_view__warp.set_subspace_roots("/")
        # - deprecated simulation view
        create_simulation_view = True
        if "cuda" in SimulationManager.get_physics_sim_device() and SimulationManager._backend == "numpy":
            SimulationManager._backend = "torch"
            carb.log_warn("Changing backend from 'numpy' to 'torch' since NumPy cannot be used with GPU piplines")
        if SimulationManager.get_backend() == "torch":
            try:
                import torch
            except ModuleNotFoundError:
                create_simulation_view = False
        if create_simulation_view:
            SimulationManager._physics_sim_view = omni.physics.tensors.create_simulation_view(
                SimulationManager.get_backend(), stage_id=stage_id
            )
            SimulationManager._physics_sim_view.set_subspace_roots("/")
        SimulationManager._physics_sim_interface.simulate(SimulationManager.get_physics_dt(), 0.0)
        SimulationManager._physics_sim_interface.fetch_results()
        # set internal states
        SimulationManager._warmup_needed = False
        SimulationManager._simulation_view_created = True
        # dispatch events
        SimulationManager._message_bus.dispatch_event(SimulationEvent.SIMULATION_SETUP.value, payload={})
        # - deprecated events
        SimulationManager._message_bus.dispatch_event(IsaacEvents.PHYSICS_WARMUP.value, payload={})
        SimulationManager._message_bus.dispatch_event(IsaacEvents.SIMULATION_VIEW_CREATED.value, payload={})
        SimulationManager._message_bus.dispatch_event(IsaacEvents.PHYSICS_READY.value, payload={})

    @classmethod
    def get_simulation_time(cls):
        return SimulationManager._simulation_manager_interface.get_simulation_time()

    @classmethod
    def get_num_physics_steps(cls):
        return SimulationManager._simulation_manager_interface.get_num_physics_steps()

    @classmethod
    def is_simulating(cls):
        return SimulationManager._simulation_manager_interface.is_simulating()

    @classmethod
    def is_paused(cls):
        return SimulationManager._simulation_manager_interface.is_paused()

    @classmethod
    def get_physics_sim_view(cls):
        return SimulationManager._physics_sim_view

    @classmethod
    def set_default_physics_scene(cls, physics_scene_prim_path: str):
        if SimulationManager._warm_start_callback is None:
            carb.log_warn("Calling set_default_physics_scene while SimulationManager is not tracking physics scenes")
            return
        if physics_scene_prim_path in SimulationManager._physics_scene_apis:
            SimulationManager._default_physics_scene_idx = list(SimulationManager._physics_scene_apis.keys()).index(
                physics_scene_prim_path
            )
        elif prim_utils.get_prim_at_path(physics_scene_prim_path).IsValid():
            prim = prim_utils.get_prim_at_path(physics_scene_prim_path)
            if prim.GetTypeName() == "PhysicsScene":
                SimulationManager._physics_scene_apis[physics_scene_prim_path] = PhysxSchema.PhysxSceneAPI.Apply(prim)
                SimulationManager._default_physics_scene_idx = list(SimulationManager._physics_scene_apis.keys()).index(
                    physics_scene_prim_path
                )
        else:
            raise Exception("physics scene specified {} doesn't exist".format(physics_scene_prim_path))

    @classmethod
    def get_default_physics_scene(cls) -> str:
        if len(SimulationManager._physics_scene_apis) > 0:
            return list(SimulationManager._physics_scene_apis.keys())[SimulationManager._default_physics_scene_idx]
        else:
            carb.log_warn("No physics scene is found in stage")
            return None

    @classmethod
    def step(
        cls, *, steps: int = 1, callback: Callable[[int, int], bool | None] | None = None, update_fabric: bool = False
    ) -> None:
        """Step the physics simulation.

        Args:
            steps: Number of steps to perform.
            callback: Optional callback function to call after each step.
                The function should take two arguments: the current step number and the total number of steps.
                If no return value is provided, the internal loop will run for the specified number of steps.
                However, if the function returns ``False``, no more steps will be performed.
            update_fabric: Whether to update fabric with the latest physics results after each step.

        Raises:
            ValueError: If the fabric is not enabled and ``update_fabric`` is set to True.
        """
        dt = cls.get_physics_dt()
        for step in range(steps):
            simulation_time = cls.get_simulation_time()
            # step physics simulation
            cls._physics_sim_interface.simulate(dt, simulation_time)
            cls._physics_sim_interface.fetch_results()
            # update fabric
            if update_fabric:
                if not cls.is_fabric_enabled():
                    raise ValueError("PhysX support for fabric is not enabled. Call '.enable_fabric()' first")
                if cls._physx_fabric_interface is None:
                    cls._physx_fabric_interface = omni.physxfabric.get_physx_fabric_interface()
                cls._physx_fabric_interface.update(simulation_time, dt)
            # call callback
            if callback is not None:
                if callback(step + 1, steps) is False:
                    break

    @classmethod
    def set_physics_sim_device(cls, val) -> None:
        if "cuda" in val:
            parsed_device = val.split(":")
            if len(parsed_device) == 1:
                device_id = SimulationManager._carb_settings.get_as_int("/physics/cudaDevice")
                if device_id < 0:
                    SimulationManager._carb_settings.set_int("/physics/cudaDevice", 0)
                    device_id = 0
            else:
                SimulationManager._carb_settings.set_int("/physics/cudaDevice", int(parsed_device[1]))
            SimulationManager._carb_settings.set_bool("/physics/suppressReadback", True)
            SimulationManager.set_broadphase_type("GPU")
            SimulationManager.enable_gpu_dynamics(flag=True)
            SimulationManager.enable_fabric(enable=True)
        elif "cpu" == val.lower():
            SimulationManager._carb_settings.set_bool("/physics/suppressReadback", False)
            # SimulationManager._carb_settings.set_int("/physics/cudaDevice", -1)
            SimulationManager.set_broadphase_type("MBP")
            SimulationManager.enable_gpu_dynamics(flag=False)
        else:
            raise Exception("Device {} is not supported.".format(val))

    @classmethod
    def get_physics_sim_device(cls) -> str:
        supress_readback = SimulationManager._carb_settings.get_as_bool("/physics/suppressReadback")
        if (not SimulationManager._physics_scene_apis and supress_readback) or (
            supress_readback
            and SimulationManager.get_broadphase_type() == "GPU"
            and SimulationManager.is_gpu_dynamics_enabled()
        ):
            device_id = SimulationManager._carb_settings.get_as_int("/physics/cudaDevice")
            if device_id < 0:
                SimulationManager._carb_settings.set_int("/physics/cudaDevice", 0)
                device_id = 0
            return f"cuda:{device_id}"
        else:
            return "cpu"

    @classmethod
    def set_physics_dt(cls, dt: float = 1.0 / 60.0, physics_scene: str = None) -> None:
        """Sets the physics dt on the physics scene provided.

        Args:
            dt (float, optional): physics dt. Defaults to 1.0/60.0.
            physics_scene (str, optional): physics scene prim path. Defaults to first physics scene found in the stage.

        Raises:
            RuntimeError: If the simulation is running/playing and dt is being set.
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
            ValueError: If the dt is not in the range [0.0, 1.0].
        """
        if app_utils.is_playing():
            raise RuntimeError("The physics dt cannot be set while the simulation is running/playing")
        if physics_scene is None:
            physics_scene_apis = SimulationManager._physics_scene_apis.values()
        else:
            physics_scene_apis = [SimulationManager._get_physics_scene_api(physics_scene=physics_scene)]

        for physics_scene_api in physics_scene_apis:
            if dt < 0:
                raise ValueError("physics dt cannot be <0")
            # if no stage or no change in physics timestep, exit.
            if stage_utils.get_current_stage(backend="usd") is None:
                return
            if dt == 0:
                physics_scene_api.GetTimeStepsPerSecondAttr().Set(0)
            elif dt > 1.0:
                raise ValueError("physics dt must be <= 1.0")
            else:
                steps_per_second = int(1.0 / dt)
                physics_scene_api.GetTimeStepsPerSecondAttr().Set(steps_per_second)
        return

    @classmethod
    def get_physics_dt(cls, physics_scene: str = None) -> str:
        """
         Returns the current physics dt.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            float: physics dt.
        """
        physics_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
        if physics_scene_api is None:
            return 1.0 / 60.0
        physics_hz = physics_scene_api.GetTimeStepsPerSecondAttr().Get()
        if physics_hz == 0:
            return 0.0
        else:
            return 1.0 / physics_hz

    @classmethod
    def get_broadphase_type(cls, physics_scene: str = None) -> str:
        """Gets current broadcast phase algorithm type.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            str: Broadcast phase algorithm used.
        """
        physics_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
        return physics_scene_api.GetBroadphaseTypeAttr().Get()

    @classmethod
    def set_broadphase_type(cls, val: str, physics_scene: str = None) -> None:
        """Broadcast phase algorithm used in simulation.

        Args:
            val (str): type of broadcasting to be used, can be "MBP".
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if physics_scene is None:
            for path, physx_scene_api in SimulationManager._physics_scene_apis.items():
                if not physx_scene_api.GetPrim().IsValid():
                    continue
                if physx_scene_api.GetBroadphaseTypeAttr().Get() is None:
                    physx_scene_api.CreateBroadphaseTypeAttr(val)
                else:
                    physx_scene_api.GetBroadphaseTypeAttr().Set(val)
        else:
            physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
            if physx_scene_api.GetBroadphaseTypeAttr().Get() is None:
                physx_scene_api.CreateBroadphaseTypeAttr(val)
            else:
                physx_scene_api.GetBroadphaseTypeAttr().Set(val)

    @classmethod
    def enable_ccd(cls, flag: bool, physics_scene: str = None) -> None:
        """Enables a second broad phase after integration that makes it possible to prevent objects from tunneling
           through each other.

        Args:
            flag (bool): enables or disables ccd on the PhysicsScene
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if physics_scene is None:
            for path, physx_scene_api in SimulationManager._physics_scene_apis.items():
                if not physx_scene_api.GetPrim().IsValid():
                    continue
                if physx_scene_api.GetEnableCCDAttr().Get() is None:
                    physx_scene_api.CreateEnableCCDAttr(flag)
                else:
                    physx_scene_api.GetEnableCCDAttr().Set(flag)
        else:
            physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
            if physx_scene_api.GetEnableCCDAttr().Get() is None:
                physx_scene_api.CreateEnableCCDAttr(flag)
            else:
                physx_scene_api.GetEnableCCDAttr().Set(flag)

    @classmethod
    def is_ccd_enabled(cls, physics_scene: str = None) -> bool:
        """Checks if ccd is enabled.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if ccd is enabled, otherwise False.
        """
        physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetEnableCCDAttr().Get()

    @classmethod
    def enable_ccd(cls, flag: bool, physics_scene: str = None) -> None:
        """Enables Continuous Collision Detection (CCD).

        Args:
            flag (bool): enables or disables CCD on the PhysicsScene.
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if flag and "cuda" in SimulationManager.get_physics_sim_device():
            carb.log_warn("CCD is not supported on GPU, ignoring request to enable it")
            return
        if physics_scene is None:
            for path, physx_scene_api in SimulationManager._physics_scene_apis.items():
                if not physx_scene_api.GetPrim().IsValid():
                    continue
                if physx_scene_api.GetEnableCCDAttr().Get() is None:
                    physx_scene_api.CreateEnableCCDAttr(flag)
                else:
                    physx_scene_api.GetEnableCCDAttr().Set(flag)
        else:
            if physics_scene in SimulationManager._physics_scene_apis:
                physx_scene_api = SimulationManager._physics_scene_apis[physics_scene]
                if physx_scene_api.GetEnableCCDAttr().Get() is None:
                    physx_scene_api.CreateEnableCCDAttr(flag)
                else:
                    physx_scene_api.GetEnableCCDAttr().Set(flag)
            else:
                raise Exception("physics scene specified {} doesn't exist".format(physics_scene))

    @classmethod
    def is_ccd_enabled(cls, physics_scene: str = None) -> bool:
        """Checks if Continuous Collision Detection (CCD) is enabled.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if CCD is enabled, otherwise False.
        """
        if physics_scene is None:
            if len(SimulationManager._physics_scene_apis) > 0:
                physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
                return physx_scene_api.GetEnableCCDAttr().Get()
            else:
                return False
        else:
            if physics_scene in SimulationManager._physics_scene_apis:
                physx_scene_api = SimulationManager._physics_scene_apis[physics_scene]
                return physx_scene_api.GetEnableCCDAttr().Get()
            else:
                raise Exception("physics scene specified {} doesn't exist".format(physics_scene))

    @classmethod
    def enable_gpu_dynamics(cls, flag: bool, physics_scene: str = None) -> None:
        """Enables gpu dynamics pipeline, required for deformables for instance.

        Args:
            flag (bool): enables or disables gpu dynamics on the PhysicsScene. If flag is true, CCD is disabled.
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if physics_scene is None:
            for path, physx_scene_api in SimulationManager._physics_scene_apis.items():
                if not physx_scene_api.GetPrim().IsValid():
                    continue
                if physx_scene_api.GetEnableGPUDynamicsAttr().Get() is None:
                    physx_scene_api.CreateEnableGPUDynamicsAttr(flag)
                else:
                    physx_scene_api.GetEnableGPUDynamicsAttr().Set(flag)
                # Disable CCD for GPU dynamics as its not supported
                if flag:
                    if SimulationManager.is_ccd_enabled():
                        carb.log_warn("Disabling CCD for GPU dynamics as its not supported")
                        SimulationManager.enable_ccd(flag=False)
        else:
            physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
            if physx_scene_api.GetEnableGPUDynamicsAttr().Get() is None:
                physx_scene_api.CreateEnableGPUDynamicsAttr(flag)
            else:
                physx_scene_api.GetEnableGPUDynamicsAttr().Set(flag)
            # Disable CCD for GPU dynamics as its not supported
            if flag:
                if SimulationManager.is_ccd_enabled(physics_scene=physics_scene):
                    carb.log_warn("Disabling CCD for GPU dynamics as its not supported")
                    SimulationManager.enable_ccd(flag=False, physics_scene=physics_scene)
            else:
                physx_scene_api.GetEnableGPUDynamicsAttr().Set(flag)

    @classmethod
    def is_gpu_dynamics_enabled(cls, physics_scene: str = None) -> bool:
        """Checks if Gpu Dynamics is enabled.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if Gpu Dynamics is enabled, otherwise False.
        """
        physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetEnableGPUDynamicsAttr().Get()

    @classmethod
    def enable_fabric(cls, enable):
        """Enables or disables physics fabric integration and associated settings.

        Args:
            enable: Whether to enable or disable fabric.
        """
        # enable/disable the omni.physx.fabric extension
        app_utils.enable_extension("omni.physx.fabric", enabled=enable)
        cls._physx_fabric_interface = omni.physxfabric.get_physx_fabric_interface() if enable else None
        # enable/disable USD updates
        SimulationManager._carb_settings.set_bool("/physics/updateToUsd", not enable)
        SimulationManager._carb_settings.set_bool("/physics/updateParticlesToUsd", not enable)
        SimulationManager._carb_settings.set_bool("/physics/updateVelocitiesToUsd", not enable)
        SimulationManager._carb_settings.set_bool("/physics/updateForceSensorsToUsd", not enable)

    @classmethod
    def is_fabric_enabled(cls):
        """Check if fabric is enabled.

        Returns:
            bool: True if fabric is enabled, otherwise False.
        """
        return app_utils.is_extension_enabled("omni.physx.fabric")

    @classmethod
    def set_solver_type(cls, solver_type: str, physics_scene: str = None) -> None:
        """solver used for simulation.

        Args:
            solver_type (str): can be "TGS" or "PGS".
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if solver_type not in ["TGS", "PGS"]:
            raise Exception("Solver type {} is not supported".format(solver_type))
        if physics_scene is None:
            for path, physx_scene_api in SimulationManager._physics_scene_apis.items():
                if not physx_scene_api.GetPrim().IsValid():
                    continue
                if physx_scene_api.GetSolverTypeAttr().Get() is None:
                    physx_scene_api.CreateSolverTypeAttr(solver_type)
                else:
                    physx_scene_api.GetSolverTypeAttr().Set(solver_type)
        else:
            physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
            if physx_scene_api.GetSolverTypeAttr().Get() is None:
                physx_scene_api.CreateSolverTypeAttr(solver_type)
            else:
                physx_scene_api.GetSolverTypeAttr().Set(solver_type)

    @classmethod
    def get_solver_type(cls, physics_scene: str = None) -> str:
        """Gets current solver type.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            str: solver used for simulation.
        """
        physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetSolverTypeAttr().Get()

    @classmethod
    def enable_stablization(cls, flag: bool, physics_scene: str = None) -> None:
        """Enables additional stabilization pass in the solver.

        Args:
            flag (bool): enables or disables stabilization on the PhysicsScene
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.
        """
        if physics_scene is None:
            for path, physx_scene_api in SimulationManager._physics_scene_apis.items():
                if not physx_scene_api.GetPrim().IsValid():
                    continue
                if physx_scene_api.GetEnableStabilizationAttr().Get() is None:
                    physx_scene_api.CreateEnableStabilizationAttr(flag)
                else:
                    physx_scene_api.GetEnableStabilizationAttr().Set(flag)
        else:
            physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
            if physx_scene_api.GetEnableStabilizationAttr().Get() is None:
                physx_scene_api.CreateEnableStabilizationAttr(flag)
            else:
                physx_scene_api.GetEnableStabilizationAttr().Set(flag)

    @classmethod
    def is_stablization_enabled(cls, physics_scene: str = None) -> bool:
        """Checks if stabilization is enabled.

        Args:
            physics_scene (str, optional): physics scene prim path.

        Raises:
            Exception: If the prim path registered in context doesn't correspond to a valid prim path currently.

        Returns:
            bool: True if stabilization is enabled, otherwise False.
        """
        physx_scene_api = SimulationManager._get_physics_scene_api(physics_scene=physics_scene)
        return physx_scene_api.GetEnableStabilizationAttr().Get()

    @classmethod
    def register_callback(
        cls, callback: Callable, event: SimulationEvent | IsaacEvents, order: int = 0, **kwargs
    ) -> int:
        """Register/subscribe a callback to be triggered when a specific simulation event occurs.

        .. warning::

            The parameter ``name`` is not longer supported. A warning message will be logged if it is defined.
            Future versions will completely remove it. At that time, defining it will result in an exception.

        Args:
            callback: The callback function to register.
            event: The simulation event to subscribe to.
            order: The subscription order.
                Callbacks registered within the same order will be triggered in the order they were registered.

        Returns:
            The unique identifier of the callback subscription.

        Raises:
            ValueError: If event is invalid.
        """
        if event not in SimulationEvent and event not in IsaacEvents:
            raise ValueError(f"Invalid simulation event: {event}. Supported events are: {list(SimulationEvent)}")
        # handle deprecations
        if "name" in kwargs:
            carb.log_warn(f"The parameter 'name' is not longer supported and will be removed in a future version")
        # check for weak reference support (when the callback is a method of a class)
        if hasattr(callback, "__self__"):
            on_event = lambda e, obj=weakref.proxy(callback.__self__): getattr(obj, callback.__name__)(e)
        else:
            on_event = callback
        # get a unique id for the callback
        uid = cls._simulation_manager_interface.get_callback_iter()
        name = f"isaacsim.core.simulation_manager:callback.{event.value}.{uid}"
        # register the callback
        # - Physics-related events
        if event in [
            IsaacEvents.PHYSICS_WARMUP,
            IsaacEvents.PHYSICS_READY,
            IsaacEvents.POST_RESET,
            IsaacEvents.SIMULATION_VIEW_CREATED,
        ]:
            cls._callbacks[uid] = cls._message_bus.observe_event(
                observer_name=name,
                event_name=event.value,
                on_event=on_event,
                order=order,
            )
        elif event in [
            SimulationEvent.PHYSICS_PRE_STEP,
            SimulationEvent.PHYSICS_POST_STEP,
            IsaacEvents.PRE_PHYSICS_STEP,
            IsaacEvents.POST_PHYSICS_STEP,
        ]:
            if hasattr(callback, "__self__"):
                on_event = lambda step_dt, context, obj=weakref.proxy(callback.__self__): (
                    getattr(obj, callback.__name__)(step_dt, context) if cls._simulation_view_created else None
                )
            else:
                on_event = lambda step_dt, context: callback(step_dt, context) if cls._simulation_view_created else None
            cls._callbacks[uid] = cls._physics_sim_interface.subscribe_physics_on_step_events(
                on_update=on_event,
                pre_step=event in [SimulationEvent.PHYSICS_PRE_STEP, IsaacEvents.PRE_PHYSICS_STEP],
                order=order,
            )
        # - Simulation lifecycle events
        elif event in [
            SimulationEvent.SIMULATION_SETUP,
            SimulationEvent.SIMULATION_STARTED,
            SimulationEvent.SIMULATION_RESUMED,
        ]:
            cls._callbacks[uid] = cls._message_bus.observe_event(
                observer_name=name,
                event_name=event.value,
                on_event=on_event,
                order=order,
            )
        elif event in [SimulationEvent.SIMULATION_PAUSED]:
            cls._callbacks[uid] = cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                event_type=int(omni.timeline.TimelineEventType.PAUSE), fn=on_event, order=order, name=name
            )
        elif event in [SimulationEvent.SIMULATION_STOPPED, IsaacEvents.TIMELINE_STOP]:
            cls._callbacks[uid] = cls._timeline.get_timeline_event_stream().create_subscription_to_pop_by_type(
                event_type=int(omni.timeline.TimelineEventType.STOP), fn=on_event, order=order, name=name
            )
        # - USD-related events
        elif event in [SimulationEvent.PRIM_DELETED, IsaacEvents.PRIM_DELETION]:
            cls._simulation_manager_interface.register_deletion_callback(on_event)
        else:
            raise RuntimeError(f"Unable to register callback for event '{event}' with uid '{uid}'")
        cls._simulation_manager_interface.set_callback_iter(uid + 1)
        return uid

    @classmethod
    def deregister_callback(cls, callback_id):
        """Deregisters a callback which was previously registered using register_callback.

        Args:
            callback_id: The ID of the callback returned by register_callback to deregister.
        """
        if callback_id in SimulationManager._callbacks:
            del SimulationManager._callbacks[callback_id]
        elif SimulationManager._simulation_manager_interface.deregister_callback(callback_id):
            return
        else:
            raise Exception("callback with id {} doesn't exist to be deregistered".format(callback_id))

    @classmethod
    def enable_usd_notice_handler(cls, flag):
        """Enables or disables the usd notice handler.

        Args:
            flag: Whether to enable or disable the handler.
        """
        SimulationManager._simulation_manager_interface.enable_usd_notice_handler(flag)
        return

    @classmethod
    def enable_fabric_usd_notice_handler(cls, stage_id, flag):
        """Enables or disables the fabric usd notice handler.

        Args:
            stage_id: The stage ID to enable or disable the handler for.
            flag: Whether to enable or disable the handler.
        """
        SimulationManager._simulation_manager_interface.enable_fabric_usd_notice_handler(stage_id, flag)
        return

    @classmethod
    def is_fabric_usd_notice_handler_enabled(cls, stage_id):
        """Checks if fabric usd notice handler is enabled.

        Args:
            stage_id: The stage ID to check.

        Returns:
            bool: True if fabric usd notice handler is enabled, otherwise False.
        """
        return SimulationManager._simulation_manager_interface.is_fabric_usd_notice_handler_enabled(stage_id)

    @classmethod
    def assets_loading(cls) -> bool:
        """Checks if textures are loaded.

        Returns:
            bool: True if textures are loading and not done yet, otherwise False.
        """
        return not SimulationManager._assets_loaded

    # Public API methods for enabling/disabling callbacks
    @classmethod
    def enable_warm_start_callback(cls, enable: bool = True) -> None:
        """Enable or disable the warm start callback.

        Args:
            enable: Whether to enable the callback.
        """
        cls._callbacks_enabled["warm_start"] = enable
        if enable:
            cls._setup_warm_start_callback()
        else:
            if cls._warm_start_callback is not None:
                cls._warm_start_callback = None

    @classmethod
    def enable_on_stop_callback(cls, enable: bool = True) -> None:
        """Enable or disable the on stop callback.

        Args:
            enable: Whether to enable the callback.
        """
        cls._callbacks_enabled["on_stop"] = enable
        if enable:
            cls._setup_on_stop_callback()
        else:
            if cls._on_stop_callback is not None:
                cls._on_stop_callback = None

    @classmethod
    def enable_post_warm_start_callback(cls, enable: bool = True) -> None:
        """Enable or disable the post warm start callback.

        .. deprecated:: 1.7.0

            |br| The :py:attr:`~isaacsim.core.simulation_manager.IsaacEvents.PHYSICS_WARMUP` event is deprecated.
            Calling this method will have no effect.

        Args:
            enable: Whether to enable the callback.
        """
        cls._callbacks_enabled["post_warm_start"] = enable

    @classmethod
    def enable_stage_open_callback(cls, enable: bool = True) -> None:
        """Enable or disable the stage open callback.
        Note: This also enables/disables the assets loading and loaded callbacks. If disabled, assets_loading() will always return True.

        Args:
            enable: Whether to enable the callback.
        """
        cls._callbacks_enabled["stage_open"] = enable
        if enable:
            cls._setup_stage_open_callback()
        else:
            if cls._stage_open_callback is not None:
                cls._stage_open_callback = None
                # Reset assets loading and loaded callbacks
                cls._assets_loaded = True
                cls._assets_loading_callback = None
                cls._assets_loaded_callback = None

    # Convenience methods for bulk operations
    @classmethod
    def enable_all_default_callbacks(cls, enable: bool = True) -> None:
        """Enable or disable all default callbacks.
        Default callbacks are: warm_start, on_stop, post_warm_start, stage_open.

        Args:
            enable: Whether to enable all callbacks.
        """
        cls.enable_warm_start_callback(enable)
        cls.enable_on_stop_callback(enable)
        cls.enable_post_warm_start_callback(enable)
        cls.enable_stage_open_callback(enable)

    @classmethod
    def is_default_callback_enabled(cls, callback_name: str) -> bool:
        """Check if a specific default callback is enabled.
        Default callbacks are: warm_start, on_stop, post_warm_start, stage_open.

        Args:
            callback_name: Name of the callback to check.

        Returns:
            Whether the callback is enabled.
        """
        return cls._callbacks_enabled.get(callback_name, False)

    @classmethod
    def get_default_callback_status(cls) -> dict:
        """Get the status of all default callbacks.
        Default callbacks are: warm_start, on_stop, post_warm_start, stage_open.

        Returns:
            Dictionary with callback names and their enabled status.
        """
        return cls._callbacks_enabled.copy()
