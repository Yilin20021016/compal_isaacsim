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

import asyncio
import os
import tempfile
from pathlib import Path

import omni.kit.test
from isaacsim.core.utils.stage import create_new_stage_async, save_stage, update_stage_async
from isaacsim.sensors.rtx import SUPPORTED_LIDAR_CONFIGS
from isaacsim.sensors.rtx.sensor_checker import ModelInfo, SensorCheckerUtil
from isaacsim.storage.native import get_assets_root_path
from pxr import Gf

from .common import create_sarcophagus


class TestSupportedLidarConfigs(omni.kit.test.AsyncTestCase):

    async def setUp(self):
        await create_new_stage_async()
        await update_stage_async()

        create_sarcophagus(enable_nonvisual_material=True)
        self._timeline = omni.timeline.get_timeline_interface()
        # Create a sensor checker
        self._checker = SensorCheckerUtil()

        # Initialize with LidarCore model
        model_info = ModelInfo()
        model_info.modelName = "LidarCore"
        model_info.modelVersion = "1.0"
        model_info.schemaVersion = "1.0"
        model_info.modelVendor = "NVIDIA"
        model_info.marketName = "GenericLidar"

        self._checker.init(model_info)

    async def tearDown(self):
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)
        await update_stage_async()


# Iterate over all supported lidar configs and variants, creating a test for each as sensor prims
DISABLE_FOR_TEST = {
    "/Isaac/Sensors/NVIDIA/Example_Solid_State.usda",
    "/Isaac/Sensors/NVIDIA/Simple_Example_Solid_State.usda",
}


def _create_lidar_parameters_test(config_name, variant):
    async def test_function(self):
        # Create sensor prim
        kwargs = {
            "path": "lidar",
            "parent": None,
            "translation": Gf.Vec3d(0.0, 0.0, 0.0),
            "orientation": Gf.Quatd(1.0, 0.0, 0.0, 0.0),
            "config": config_name,
            "variant": variant,
            "omni:sensor:Core:outputFrameOfReference": "WORLD",
        }

        _, self.sensor = omni.kit.commands.execute(f"IsaacSensorCreateRtxLidar", **kwargs)
        sensor_type = self.sensor.GetTypeName()
        self.assertEqual(
            sensor_type, "OmniLidar", f"Expected OmniLidar prim, got {sensor_type}. Was sensor prim created?"
        )

        # Get the prim path
        prim_path = str(self.sensor.GetPath())

        # Save the current stage to a temporary file
        with tempfile.TemporaryDirectory() as temp_dir:
            temp_usd_path = os.path.join(temp_dir, "test_stage.usda")
            save_result = save_stage(temp_usd_path, save_and_reload_in_place=False)
            self.assertTrue(save_result, f"Failed to save stage to {temp_usd_path}")

            # Validate parameters using sensor checker
            error_string = self._checker.validateParams(temp_usd_path, prim_path)
            self.assertIsNone(
                error_string,
                f"Sensor parameter validation failed for config {config_name} variant {variant}: {error_string}",
            )

            # Verify that validated parameters were cached
            validated_params = self._checker.getValidatedParams()
            self.assertIsNotNone(validated_params, "Failed to retrieve validated parameters")
            self.assertGreater(
                validated_params.numParams,
                0,
                f"Expected validated parameters, got {validated_params.numParams} parameters",
            )

    return test_function


for config_path in SUPPORTED_LIDAR_CONFIGS:
    if config_path in DISABLE_FOR_TEST:
        continue
    config_name = Path(config_path).stem
    for variant in SUPPORTED_LIDAR_CONFIGS[config_path] or [None]:
        test_name = f"test_lidar_parameters_{config_name}_{variant}"
        test_func = _create_lidar_parameters_test(config_name, variant)
        test_func.__name__ = test_name
        test_func.__doc__ = (
            f"Test supported lidar configs and variants, with config {config_name} and variant {variant}."
        )
        setattr(TestSupportedLidarConfigs, test_name, test_func)
