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
from enum import Enum

import carb
import pxr
from pxr import Sdf, Usd


class Classes(Enum):

    ROBOT_API = "IsaacRobotAPI"
    LINK_API = "IsaacLinkAPI"
    REFERENCE_POINT_API = "IsaacReferencePointAPI"
    JOINT_API = "IsaacJointAPI"
    SURFACE_GRIPPER = "IsaacSurfaceGripper"
    ATTACHMENT_POINT_API = "IsaacAttachmentPointAPI"


_attr_prefix = "isaac"


class DofOffsetOpOrder(Enum):

    TRANS_X = "TransX"
    TRANS_Y = "TransY"
    TRANS_Z = "TransZ"
    ROT_X = "RotX"
    ROT_Y = "RotY"
    ROT_Z = "RotZ"


class Attributes(Enum):
    DESCRIPTION = (f"{_attr_prefix}:description", "Description", pxr.Sdf.ValueTypeNames.String)
    NAMESPACE = (f"{_attr_prefix}:namespace", "Namespace", pxr.Sdf.ValueTypeNames.String)
    ROBOT_TYPE = (f"{_attr_prefix}:robotType", "Robot Type", pxr.Sdf.ValueTypeNames.Token)
    LICENSE = (f"{_attr_prefix}:license", "License", pxr.Sdf.ValueTypeNames.Token)
    VERSION = (f"{_attr_prefix}:version", "Version", pxr.Sdf.ValueTypeNames.String)
    SOURCE = (f"{_attr_prefix}:source", "Source", pxr.Sdf.ValueTypeNames.String)
    CHANGELOG = (f"{_attr_prefix}:changelog", "Changelog", pxr.Sdf.ValueTypeNames.StringArray)
    NAME_OVERRIDE = (f"{_attr_prefix}:nameOverride", "Name Override", pxr.Sdf.ValueTypeNames.String)
    REFERENCE_DESCRIPTION = (f"{_attr_prefix}:Description", "Reference Description", pxr.Sdf.ValueTypeNames.String)
    FORWARD_AXIS = (f"{_attr_prefix}:forwardAxis", "Forward Axis", pxr.Sdf.ValueTypeNames.Token)
    JOINT_NAME_OVERRIDE = (f"{_attr_prefix}:NameOverride", "Joint Name Override", pxr.Sdf.ValueTypeNames.String)
    DOF_OFFSET_OP_ORDER = (
        f"{_attr_prefix}:physics:DofOffsetOpOrder",
        "Dof Offset Op Order",
        pxr.Sdf.ValueTypeNames.TokenArray,
    )
    ACTUATOR = (f"{_attr_prefix}:actuator", "Actuator", pxr.Sdf.ValueTypeNames.BoolArray)
    RETRY_INTERVAL = (f"{_attr_prefix}:retryInterval", "Retry Interval", pxr.Sdf.ValueTypeNames.Float)
    STATUS = (f"{_attr_prefix}:status", "Status", pxr.Sdf.ValueTypeNames.Token)
    SHEAR_FORCE_LIMIT = (f"{_attr_prefix}:shearForceLimit", "Shear Force Limit", pxr.Sdf.ValueTypeNames.Float)
    COAXIAL_FORCE_LIMIT = (f"{_attr_prefix}:coaxialForceLimit", "Coaxial Force Limit", pxr.Sdf.ValueTypeNames.Float)
    MAX_GRIP_DISTANCE = (f"{_attr_prefix}:maxGripDistance", "Max Grip Distance", pxr.Sdf.ValueTypeNames.Float)
    CLEARANCE_OFFSET = (f"{_attr_prefix}:clearanceOffset", "Clearance Offset", pxr.Sdf.ValueTypeNames.Float)

    # Custom properties for name and type
    @property
    def name(self):
        return self.value[0]

    @property
    def display_name(self):
        return self.value[1]

    @property
    def type(self):
        return self.value[2]


class Relations(Enum):
    ROBOT_LINKS = (f"{_attr_prefix}:physics:robotLinks", "Robot Links")
    ROBOT_JOINTS = (f"{_attr_prefix}:physics:robotJoints", "Robot Joints")
    ATTACHMENT_POINTS = (f"{_attr_prefix}:attachmentPoints", "Attachment Points")
    GRIPPED_OBJECTS = (f"{_attr_prefix}:grippedObjects", "Gripped Objects")

    @property
    def name(self):
        return self.value[0]

    @property
    def display_name(self):
        return self.value[1]


def _create_attributes(prim: pxr.Usd.Prim, attributes, write_sparsely: bool = True):
    for attr in attributes:
        prim.CreateAttribute(attr.name, attr.type, write_sparsely)


def _create_relationships(prim: pxr.Usd.Prim, relationships, custom: bool = True):
    for rel in relationships:
        prim.CreateRelationship(rel.name, custom=custom)


def _apply_api(
    prim: pxr.Usd.Prim,
    schema: Classes,
    *,
    attributes=(),
    relationships=(),
    write_sparsely: bool = True,
    relationships_custom: bool = True,
):
    prim.AddAppliedSchema(schema.value)
    _create_attributes(prim, attributes, write_sparsely)
    _create_relationships(prim, relationships, relationships_custom)


def ApplyRobotAPI(prim: pxr.Usd.Prim):
    _apply_api(
        prim,
        Classes.ROBOT_API,
        attributes=[
            Attributes.DESCRIPTION,
            Attributes.NAMESPACE,
            Attributes.ROBOT_TYPE,
            Attributes.LICENSE,
            Attributes.VERSION,
            Attributes.SOURCE,
            Attributes.CHANGELOG,
        ],
        relationships=[Relations.ROBOT_LINKS, Relations.ROBOT_JOINTS],
    )

    stage = prim.GetStage()
    if stage:
        from . import utils as _robot_schema_utils

        _robot_schema_utils.PopulateRobotSchemaFromArticulation(stage, prim, prim)


def ApplyLinkAPI(prim: pxr.Usd.Prim):
    _apply_api(prim, Classes.LINK_API, attributes=[Attributes.NAME_OVERRIDE])


def ApplyReferencePointAPI(prim: pxr.Usd.Prim):
    carb.log_warn("ApplyReferencePointAPI is deprecated. Use ApplySiteAPI instead.")
    _apply_api(prim, Classes.SITE_API)
    # for attr in [Attributes.REFERENCE_DESCRIPTION, Attributes.FORWARD_AXIS]:
    #     prim.CreateAttribute(attr.name, attr.type, True)


def ApplyJointAPI(prim: pxr.Usd.Prim):
    _apply_api(prim, Classes.JOINT_API)
    # for attr in [
    #     Attributes.DOF_OFFSET_OP_ORDER,
    #     Attributes.JOINT_NAME_OVERRIDE,
    #     Attributes.ACTUATOR,
    # ]:
    #     prim.CreateAttribute(attr.name, attr.type, True)


def CreateSurfaceGripper(stage: pxr.Usd.Stage, prim_path: str) -> pxr.Usd.Prim:
    """Creates a Surface Gripper prim with all its attributes and relationships.

    Args:
        stage: The USD stage to create the prim in
        prim_path: The path where to create the prim

    Returns:
        The created Surface Gripper prim
    """
    # Create the prim
    prim = stage.DefinePrim(prim_path, Classes.SURFACE_GRIPPER.value)

    # Create attributes with default values
    _create_attributes(
        prim,
        [
            Attributes.STATUS,
            Attributes.SHEAR_FORCE_LIMIT,
            Attributes.COAXIAL_FORCE_LIMIT,
            Attributes.MAX_GRIP_DISTANCE,
            Attributes.RETRY_INTERVAL,
        ],
        write_sparsely=False,
    )

    # Create relationships
    _create_relationships(
        prim,
        [Relations.ATTACHMENT_POINTS, Relations.GRIPPED_OBJECTS],
        custom=False,
    )

    return prim


def ApplyAttachmentPointAPI(prim: pxr.Usd.Prim):
    _apply_api(prim, Classes.ATTACHMENT_POINT_API)
    # for attr in [Attributes.FORWARD_AXIS, Attributes.CLEARANCE_OFFSET]:
    #     prim.CreateAttribute(attr.name, attr.type, False)
