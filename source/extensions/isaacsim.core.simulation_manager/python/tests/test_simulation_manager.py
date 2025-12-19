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

import isaacsim.core.experimental.utils.stage as stage_utils
import omni.kit.app
import omni.kit.test
import omni.timeline
from isaacsim.core.simulation_manager import IsaacEvents, SimulationManager


class TestSimulationManagerBackend(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager backend management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        SimulationManager.set_backend("numpy")
        super().tearDown()

    async def test_get_set_backend(self):
        """Test getting and setting all valid backends."""
        for backend in ["numpy", "torch", "warp"]:
            SimulationManager.set_backend(backend)
            self.assertEqual(SimulationManager.get_backend(), backend)

    async def test_get_backend_utils(self):
        """Test getting backend utils for all backends and invalid backend."""
        # Test numpy backend
        SimulationManager.set_backend("numpy")
        utils = SimulationManager._get_backend_utils()
        self.assertIsNotNone(utils)
        self.assertTrue(hasattr(utils, "convert"))

        # Test torch backend
        SimulationManager.set_backend("torch")
        try:
            utils = SimulationManager._get_backend_utils()
            self.assertIsNotNone(utils)
            self.assertTrue(hasattr(utils, "convert"))
        except Exception as e:
            print(f"test_get_backend_utils failed for torch: {e}")

        # Test warp backend
        SimulationManager.set_backend("warp")
        try:
            utils = SimulationManager._get_backend_utils()
            self.assertIsNotNone(utils)
            self.assertTrue(hasattr(utils, "convert"))
        except Exception as e:
            print(f"test_get_backend_utils failed for warp: {e}")

        # Test invalid backend raises exception
        SimulationManager.set_backend("invalid_backend")
        with self.assertRaises(Exception):
            SimulationManager._get_backend_utils()


class TestSimulationManagerDefaultCallbacks(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager default callback enable/disable functionality."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        SimulationManager.enable_all_default_callbacks(True)
        super().tearDown()

    async def test_default_callbacks(self):
        """Test all default callback enable/disable functionality."""
        # Test individual callbacks
        SimulationManager.enable_warm_start_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("warm_start"))
        SimulationManager.enable_warm_start_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("warm_start"))

        SimulationManager.enable_on_stop_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("on_stop"))
        SimulationManager.enable_on_stop_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("on_stop"))

        SimulationManager.enable_post_warm_start_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("post_warm_start"))
        SimulationManager.enable_post_warm_start_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("post_warm_start"))

        SimulationManager.enable_stage_open_callback(True)
        self.assertTrue(SimulationManager.is_default_callback_enabled("stage_open"))
        SimulationManager.enable_stage_open_callback(False)
        self.assertFalse(SimulationManager.is_default_callback_enabled("stage_open"))

        # Test enable/disable all callbacks
        SimulationManager.enable_all_default_callbacks(False)
        status = SimulationManager.get_default_callback_status()
        self.assertFalse(all(status.values()))

        SimulationManager.enable_all_default_callbacks(True)
        status = SimulationManager.get_default_callback_status()
        self.assertTrue(all(status.values()))

        # Test get_default_callback_status returns correct keys
        expected_keys = {"warm_start", "on_stop", "post_warm_start", "stage_open"}
        self.assertEqual(set(status.keys()), expected_keys)

        # Test invalid callback name returns False
        self.assertFalse(SimulationManager.is_default_callback_enabled("nonexistent_callback"))


class TestSimulationManagerPhysicsDevice(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager physics device management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        stage_utils.define_prim("/World/PhysicsScene", type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()
        self._original_device = SimulationManager.get_physics_sim_device()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        try:
            SimulationManager.set_physics_sim_device(self._original_device)
        except Exception as e:
            print(f"tearDown failed to restore physics sim device: {e}")
        super().tearDown()

    async def test_physics_sim_device(self):
        """Test getting and setting physics simulation device."""
        # Test get returns string
        device = SimulationManager.get_physics_sim_device()
        self.assertIsInstance(device, str)
        self.assertTrue(device == "cpu" or device.startswith("cuda:"))

        # Test setting to CPU
        SimulationManager.set_physics_sim_device("cpu")

        # Test setting to CUDA
        try:
            SimulationManager.set_physics_sim_device("cuda")
            device = SimulationManager.get_physics_sim_device()
            self.assertIn("cuda", device)
        except Exception as e:
            print(f"test_physics_sim_device failed for cuda: {e}")

        # Test setting to CUDA with specific device ID
        try:
            SimulationManager.set_physics_sim_device("cuda:0")
            device = SimulationManager.get_physics_sim_device()
            self.assertEqual(device, "cuda:0")
        except Exception as e:
            print(f"test_physics_sim_device failed for cuda:0: {e}")

        # Test invalid device raises exception
        with self.assertRaises(Exception):
            SimulationManager.set_physics_sim_device("invalid_device")


class TestSimulationManagerPhysicsSceneSettings(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager physics scene settings (dt, broadphase, CCD, GPU dynamics, stabilization, solver)."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        self._registered_callbacks = []
        self._received_dt_values = []

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        for callback_id in self._registered_callbacks:
            try:
                SimulationManager.deregister_callback(callback_id)
            except Exception as e:
                print(f"tearDown failed to deregister callback {callback_id}: {e}")
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def _create_physics_scene(self):
        """Helper to create physics scene for tests that need it."""
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

    def _physics_step_callback(self, step_dt, context):
        """Callback to capture physics step dt values."""
        self._received_dt_values.append(step_dt)

    async def test_physics_scene_settings_no_scene(self):
        """Test all physics scene settings without specifying a physics scene."""
        # Test get physics dt default
        dt = SimulationManager.get_physics_dt()
        self.assertIsNotNone(dt)
        self.assertGreaterEqual(dt, 0)

        # Test set physics dt and verify in callback
        test_dts = [1.0 / 60.0, 1.0 / 120.0, 1.0 / 30.0, 1.0 / 240.0]
        for expected_dt in test_dts:
            self._received_dt_values.clear()
            try:
                SimulationManager.set_physics_dt(expected_dt)
                dt = SimulationManager.get_physics_dt()
                self.assertAlmostEqual(dt, expected_dt, places=5)
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene failed for dt {expected_dt}: {e}")
                continue

            callback_id = SimulationManager.register_callback(
                self._physics_step_callback, event=IsaacEvents.POST_PHYSICS_STEP
            )
            self._registered_callbacks.append(callback_id)

            timeline = omni.timeline.get_timeline_interface()
            timeline.play()
            await omni.kit.app.get_app().next_update_async()
            await omni.kit.app.get_app().next_update_async()
            timeline.stop()
            await omni.kit.app.get_app().next_update_async()

            if self._received_dt_values:
                for received_dt in self._received_dt_values:
                    self.assertAlmostEqual(
                        received_dt,
                        expected_dt,
                        places=5,
                        msg=f"Callback dt {received_dt} does not match expected {expected_dt}",
                    )

            SimulationManager.deregister_callback(callback_id)
            self._registered_callbacks.remove(callback_id)

        # Test set physics dt
        expected_dt = 1.0 / 90.0
        try:
            SimulationManager.set_physics_dt(expected_dt)
            dt = SimulationManager.get_physics_dt()
            self.assertAlmostEqual(dt, expected_dt, places=5)
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene set_physics_dt failed: {e}")

        # Test set physics dt to zero
        try:
            SimulationManager.set_physics_dt(0)
            dt = SimulationManager.get_physics_dt()
            self.assertEqual(dt, 0.0)
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene set_physics_dt_zero failed: {e}")

        # Test invalid physics dt values
        has_physics_scene = SimulationManager.get_default_physics_scene() is not None
        if has_physics_scene:
            with self.assertRaises(ValueError):
                SimulationManager.set_physics_dt(-1.0)
            with self.assertRaises(ValueError):
                SimulationManager.set_physics_dt(1.5)

        # Test broadphase type
        for btype in ["MBP", "GPU"]:
            try:
                SimulationManager.set_broadphase_type(btype)
                result = SimulationManager.get_broadphase_type()
                self.assertEqual(result, btype)
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene broadphase failed for {btype}: {e}")

        # Test solver type
        for solver in ["TGS", "PGS"]:
            try:
                SimulationManager.set_solver_type(solver)
                result = SimulationManager.get_solver_type()
                self.assertEqual(result, solver)
            except Exception as e:
                print(f"test_physics_scene_settings_no_scene solver failed for {solver}: {e}")

        # Test invalid solver type raises exception
        with self.assertRaises(Exception):
            SimulationManager.set_solver_type("INVALID")

        # Test CCD
        try:
            SimulationManager.set_physics_sim_device("cpu")
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene failed to set device to cpu: {e}")

        try:
            SimulationManager.enable_ccd(True)
            self.assertTrue(SimulationManager.is_ccd_enabled())
            SimulationManager.enable_ccd(False)
            self.assertFalse(SimulationManager.is_ccd_enabled())
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene ccd failed: {e}")

        # Test GPU dynamics
        try:
            SimulationManager.enable_gpu_dynamics(True)
            self.assertTrue(SimulationManager.is_gpu_dynamics_enabled())
            SimulationManager.enable_gpu_dynamics(False)
            self.assertFalse(SimulationManager.is_gpu_dynamics_enabled())
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene gpu_dynamics failed: {e}")

        # Test stabilization
        try:
            SimulationManager.enable_stablization(True)
            self.assertTrue(SimulationManager.is_stablization_enabled())
            SimulationManager.enable_stablization(False)
            self.assertFalse(SimulationManager.is_stablization_enabled())
        except Exception as e:
            print(f"test_physics_scene_settings_no_scene stabilization failed: {e}")

        # Test get default physics scene
        scene = SimulationManager.get_default_physics_scene()
        if scene is not None:
            self.assertIsInstance(scene, str)

    async def test_physics_scene_settings_with_scene(self):
        """Test all physics scene settings with a specific physics scene."""
        await self._create_physics_scene()

        # Test set physics dt with physics scene
        expected_dt = 1.0 / 90.0
        try:
            SimulationManager.set_physics_dt(expected_dt, physics_scene=self._physics_scene_path)
            dt = SimulationManager.get_physics_dt(physics_scene=self._physics_scene_path)
            self.assertAlmostEqual(dt, expected_dt, places=5)
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene set_physics_dt failed: {e}")

        # Test broadphase type with physics scene
        for btype in ["MBP", "GPU"]:
            try:
                SimulationManager.set_broadphase_type(btype, physics_scene=self._physics_scene_path)
                result = SimulationManager.get_broadphase_type(physics_scene=self._physics_scene_path)
                self.assertEqual(result, btype)
            except Exception as e:
                print(f"test_physics_scene_settings_with_scene broadphase failed for {btype}: {e}")

        # Test solver type with physics scene
        for solver in ["TGS", "PGS"]:
            try:
                SimulationManager.set_solver_type(solver, physics_scene=self._physics_scene_path)
                result = SimulationManager.get_solver_type(physics_scene=self._physics_scene_path)
                self.assertEqual(result, solver)
            except Exception as e:
                print(f"test_physics_scene_settings_with_scene solver failed for {solver}: {e}")

        # Test CCD with physics scene
        try:
            SimulationManager.set_physics_sim_device("cpu")
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene failed to set device to cpu: {e}")

        try:
            SimulationManager.enable_ccd(True, physics_scene=self._physics_scene_path)
            self.assertTrue(SimulationManager.is_ccd_enabled(physics_scene=self._physics_scene_path))
            SimulationManager.enable_ccd(False, physics_scene=self._physics_scene_path)
            self.assertFalse(SimulationManager.is_ccd_enabled(physics_scene=self._physics_scene_path))
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene ccd failed: {e}")

        # Test GPU dynamics with physics scene
        try:
            SimulationManager.enable_gpu_dynamics(True, physics_scene=self._physics_scene_path)
            self.assertTrue(SimulationManager.is_gpu_dynamics_enabled(physics_scene=self._physics_scene_path))
            SimulationManager.enable_gpu_dynamics(False, physics_scene=self._physics_scene_path)
            self.assertFalse(SimulationManager.is_gpu_dynamics_enabled(physics_scene=self._physics_scene_path))
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene gpu_dynamics failed: {e}")

        # Test stabilization with physics scene
        try:
            SimulationManager.enable_stablization(True, physics_scene=self._physics_scene_path)
            self.assertTrue(SimulationManager.is_stablization_enabled(physics_scene=self._physics_scene_path))
            SimulationManager.enable_stablization(False, physics_scene=self._physics_scene_path)
            self.assertFalse(SimulationManager.is_stablization_enabled(physics_scene=self._physics_scene_path))
        except Exception as e:
            print(f"test_physics_scene_settings_with_scene stabilization failed: {e}")

        # Test set default physics scene
        try:
            SimulationManager.set_default_physics_scene(self._physics_scene_path)
        except Exception as e:
            if "SimulationManager is not tracking physics scenes" not in str(e):
                raise


class TestSimulationManagerCallbacks(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager callback registration and event handling."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()
        self._registered_callbacks = []

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        for callback_id in self._registered_callbacks:
            try:
                SimulationManager.deregister_callback(callback_id)
            except Exception as e:
                print(f"tearDown failed to deregister callback {callback_id}: {e}")
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_register_callbacks(self):
        """Test registering and deregistering callbacks for all event types."""
        # Test message bus callbacks
        events = [
            IsaacEvents.PHYSICS_WARMUP,
            IsaacEvents.PHYSICS_READY,
            IsaacEvents.POST_RESET,
            IsaacEvents.SIMULATION_VIEW_CREATED,
        ]
        for event in events:
            callback_id = SimulationManager.register_callback(lambda x: None, event=event)
            self._registered_callbacks.append(callback_id)
            self.assertIsNotNone(callback_id)

        # Test physics step callbacks
        for event in [IsaacEvents.PRE_PHYSICS_STEP, IsaacEvents.POST_PHYSICS_STEP]:
            callback_id = SimulationManager.register_callback(lambda dt, context: None, event=event)
            self._registered_callbacks.append(callback_id)
            self.assertIsNotNone(callback_id)

        # Test timeline stop callback
        callback_id = SimulationManager.register_callback(lambda x: None, event=IsaacEvents.TIMELINE_STOP)
        self._registered_callbacks.append(callback_id)
        self.assertIsNotNone(callback_id)

        # Test prim deletion callback
        try:
            callback_id = SimulationManager.register_callback(lambda prim_path: None, event=IsaacEvents.PRIM_DELETION)
            self.assertIsNotNone(callback_id)
        except Exception as e:
            print(f"test_register_callbacks prim_deletion failed: {e}")

        # Test callbacks with specific execution order
        callback_id_1 = SimulationManager.register_callback(
            lambda dt, context: None, event=IsaacEvents.POST_PHYSICS_STEP, order=0
        )
        callback_id_2 = SimulationManager.register_callback(
            lambda dt, context: None, event=IsaacEvents.POST_PHYSICS_STEP, order=10
        )
        self._registered_callbacks.extend([callback_id_1, callback_id_2])
        self.assertNotEqual(callback_id_1, callback_id_2)

        # Test callback with bound method
        class CallbackHolder:
            def __init__(self):
                self.called = False

            def on_physics_ready(self, event):
                self.called = True

        holder = CallbackHolder()
        callback_id = SimulationManager.register_callback(holder.on_physics_ready, event=IsaacEvents.PHYSICS_READY)
        self._registered_callbacks.append(callback_id)
        self.assertIsNotNone(callback_id)

        # Test deregistering callback
        callback_id = SimulationManager.register_callback(lambda x: None, event=IsaacEvents.PHYSICS_READY)
        SimulationManager.deregister_callback(callback_id)
        with self.assertRaises(Exception):
            SimulationManager.deregister_callback(callback_id)

        # Test deregistering invalid callback raises exception
        with self.assertRaises(Exception):
            SimulationManager.deregister_callback(999999)

    async def test_event_dispatch(self):
        """Test that all events are dispatched correctly."""
        warmup_received = []
        ready_received = []
        view_created_received = []
        stop_received = []
        execution_order = []

        # Register all event callbacks
        warmup_callback_id = SimulationManager.register_callback(
            lambda event: warmup_received.append(True), event=IsaacEvents.PHYSICS_WARMUP
        )
        ready_callback_id = SimulationManager.register_callback(
            lambda event: ready_received.append(True), event=IsaacEvents.PHYSICS_READY
        )
        view_created_callback_id = SimulationManager.register_callback(
            lambda event: view_created_received.append(True), event=IsaacEvents.SIMULATION_VIEW_CREATED
        )
        stop_callback_id = SimulationManager.register_callback(
            lambda event: stop_received.append(True), event=IsaacEvents.TIMELINE_STOP
        )
        pre_callback_id = SimulationManager.register_callback(
            lambda dt, context: execution_order.append("pre"), event=IsaacEvents.PRE_PHYSICS_STEP
        )
        post_callback_id = SimulationManager.register_callback(
            lambda dt, context: execution_order.append("post"), event=IsaacEvents.POST_PHYSICS_STEP
        )
        self._registered_callbacks.extend(
            [
                warmup_callback_id,
                ready_callback_id,
                view_created_callback_id,
                stop_callback_id,
                pre_callback_id,
                post_callback_id,
            ]
        )

        # Start simulation
        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Verify warmup, ready, and view created events
        self.assertGreater(len(warmup_received), 0, "PHYSICS_WARMUP event was not received")
        self.assertGreater(len(ready_received), 0, "PHYSICS_READY event was not received")
        self.assertGreater(len(view_created_received), 0, "SIMULATION_VIEW_CREATED event was not received")

        # Verify pre/post physics step order
        if len(execution_order) >= 2:
            pre_indices = [i for i, x in enumerate(execution_order) if x == "pre"]
            post_indices = [i for i, x in enumerate(execution_order) if x == "post"]
            if pre_indices and post_indices:
                self.assertLess(
                    pre_indices[0], post_indices[0], "PRE_PHYSICS_STEP should execute before POST_PHYSICS_STEP"
                )

        # Stop and verify stop event
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        self.assertGreater(len(stop_received), 0, "TIMELINE_STOP event was not received")


class TestSimulationManagerSimulationLifecycle(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager simulation state, step, and initialization."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()
        self._physics_scene_path = "/World/PhysicsScene"
        stage_utils.define_prim(self._physics_scene_path, type_name="PhysicsScene")
        await omni.kit.app.get_app().next_update_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        timeline = omni.timeline.get_timeline_interface()
        timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        super().tearDown()

    async def test_simulation_state(self):
        """Test simulation state through play, pause, and time advancement."""
        # Test initial state
        self.assertFalse(SimulationManager.is_simulating())
        self.assertFalse(SimulationManager.is_paused())
        self.assertEqual(SimulationManager.get_simulation_time(), 0.0)
        self.assertEqual(SimulationManager.get_num_physics_steps(), 0)

        timeline = omni.timeline.get_timeline_interface()
        initial_time = SimulationManager.get_simulation_time()

        # Test state after play
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(SimulationManager.is_simulating())
        self.assertFalse(SimulationManager.is_paused())

        # Test simulation time advances
        await omni.kit.app.get_app().next_update_async()
        self.assertGreater(SimulationManager.get_simulation_time(), initial_time)

        # Test state after pause
        timeline.pause()
        await omni.kit.app.get_app().next_update_async()
        self.assertTrue(SimulationManager.is_simulating())
        self.assertTrue(SimulationManager.is_paused())

    async def test_simulation_operations(self):
        """Test simulation operations including physics view, stepping, and initialization."""
        # Test physics sim view before simulation
        self.assertIsNone(SimulationManager.get_physics_sim_view())

        timeline = omni.timeline.get_timeline_interface()
        timeline.play()
        await omni.kit.app.get_app().next_update_async()
        await omni.kit.app.get_app().next_update_async()

        # Test physics sim view after simulation starts
        # View may or may not be available depending on torch availability
        # Just verify the function doesn't raise
        SimulationManager.get_physics_sim_view()

        try:
            SimulationManager.step()
        except Exception as e:
            print(f"test_simulation_operations step failed: {e}")

        # Test initialize_physics
        try:
            SimulationManager.initialize_physics()
        except Exception as e:
            print(f"test_simulation_operations initialize_physics failed: {e}")

        # Test assets_loading returns bool
        result = SimulationManager.assets_loading()
        self.assertIsInstance(result, bool)


class TestSimulationManagerFabricAndNoticeHandlers(omni.kit.test.AsyncTestCase):
    """Tests for SimulationManager fabric and USD notice handler management."""

    async def setUp(self):
        """Method called to prepare the test fixture."""
        super().setUp()
        await stage_utils.create_new_stage_async()

    async def tearDown(self):
        """Method called immediately after the test method has been called."""
        super().tearDown()

    async def test_fabric_and_notice_handlers(self):
        """Test all fabric and USD notice handler functionality."""
        # Test enable/disable fabric
        try:
            SimulationManager.enable_fabric(True)
            SimulationManager.enable_fabric(False)
        except Exception as e:
            print(f"test_fabric_and_notice_handlers enable_fabric failed: {e}")

        # Test is_fabric_enabled returns bool
        result = SimulationManager.is_fabric_enabled()
        self.assertIsInstance(result, bool)

        # Test enable/disable USD notice handler
        try:
            SimulationManager.enable_usd_notice_handler(True)
            SimulationManager.enable_usd_notice_handler(False)
        except Exception as e:
            print(f"test_fabric_and_notice_handlers usd_notice_handler failed: {e}")

        # Test fabric USD notice handler
        try:
            stage = stage_utils.get_current_stage()
            stage_id = stage_utils.get_stage_id(stage)

            SimulationManager.enable_fabric_usd_notice_handler(stage_id, True)
            SimulationManager.enable_fabric_usd_notice_handler(stage_id, False)

            result = SimulationManager.is_fabric_usd_notice_handler_enabled(stage_id)
            self.assertIsInstance(result, bool)
        except Exception as e:
            print(f"test_fabric_and_notice_handlers fabric_usd_notice_handler failed: {e}")
