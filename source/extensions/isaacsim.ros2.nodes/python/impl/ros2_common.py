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

"""Common utilities and constants for ROS 2 nodes extension."""

from typing import List

import numpy as np
import omni.replicator.core as rep
import omni.syntheticdata
from isaacsim.core.nodes.scripts.utils import (
    register_annotator_from_node_with_telemetry,
    register_node_writer_with_telemetry,
)

# Nodes extension constants
BRIDGE_NAME = "isaacsim.ros2.bridge"
BRIDGE_PREFIX = "ROS2"


def build_rtx_sensor_pointcloud_writer(
    metadata: List[str], enable_full_scan: bool = True, use_system_time: bool = False
) -> rep.Writer:
    """Build and register an RTX sensor point cloud writer with specified metadata.

    Dynamically creates and registers a custom annotator and writer for publishing RTX sensor
    point cloud data with the specified metadata fields. The annotator and writer are cached
    and reused if they already exist with the same configuration.

    Args:
        param metadata: List of metadata field names to include in the point cloud output.
            Valid options include "intensity", "timestamp", "emitterId", "channelId",
            "materialId", "tickId", "hitNormal", "velocity", "objectId", "echoId",
            "tickState", and "radialVelocityMS".
        param enable_full_scan: If True, enables full scan buffer mode. If False, enables
            per-frame output mode.
        param use_system_time: If True, uses system time for timestamps. If False, uses
            simulation time.

    Returns:
        The registered replicator writer instance configured for publishing RTX sensor
        point cloud data with the specified metadata and timing configuration.

    Example:

    .. code-block:: python

        >>> from isaacsim.ros2.nodes.python.impl.ros2_common import build_rtx_sensor_pointcloud_writer
        >>>
        >>> writer = build_rtx_sensor_pointcloud_writer(
        ...     metadata=["intensity", "objectId"],
        ...     enable_full_scan=True,
        ...     use_system_time=False,
        ... )
        >>>
        >>> writer.initialize(
        ...     frameId="lidar_frame",
        ...     topicName="point_cloud",
        ... )
    """

    # Dynamically name, build, and register a new Annotator based on the selected metadata if it doesn't exist yet
    annotator_name = "IsaacCreateRTXLidarScanBuffer"
    annotator_name += "PerFrame" if not enable_full_scan else ""
    annotator_name += "_".join(metadata)
    if annotator_name not in rep.AnnotatorRegistry.get_registered_annotators():
        init_params = {"enablePerFrameOutput": not enable_full_scan}
        for metadata_item in metadata:
            init_params[f"output{metadata_item[0].upper()}{metadata_item[1:]}"] = True
        register_annotator_from_node_with_telemetry(
            name=annotator_name,
            input_rendervars=[
                "GenericModelOutputPtr",
            ],
            node_type_id="isaacsim.sensors.rtx.IsaacCreateRTXLidarScanBuffer",
            init_params=init_params,
            output_data_type=np.float32,
            output_channels=3,
        )

    # Dynamically name, build, and register a new Writer based on the selected metadata if it doesn't exist yet
    time_type = "SystemTime" if use_system_time else ""
    time_node_type = "system" if use_system_time else "simulation"

    writer_name = "RtxLidar" + f"ROS2{time_type}PublishPointCloud"
    writer_name += "Buffer" if enable_full_scan else ""
    writer_name += "_".join(metadata)
    if writer_name not in rep.WriterRegistry.get_writers():
        register_node_writer_with_telemetry(
            name=writer_name,
            node_type_id="isaacsim.ros2.bridge.ROS2PublishPointCloud",
            annotators=[
                annotator_name,
                "PostProcessDispatchIsaacSimulationGate",
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    f"IsaacRead{time_node_type.capitalize()}Time",
                    attributes_mapping={f"outputs:{time_node_type}Time": "inputs:timeStamp"},
                ),
            ],
            category="isaacsim.ros2.bridge",
        )

    writer = rep.writers.get(writer_name)
    return writer
