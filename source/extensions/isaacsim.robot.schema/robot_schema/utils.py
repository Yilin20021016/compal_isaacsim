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
from __future__ import annotations

from collections import deque
from collections.abc import Callable

import carb
import omni
import pxr
from usd.schema.isaac.robot_schema import *

_DEPRECATED_DOF_ATTRS: tuple[tuple[str, str, pxr.TfToken], ...] = (
    ("isaac:physics:Tr_X:DoFOffset", "TransX", pxr.UsdPhysics.Tokens.transX),
    ("isaac:physics:Tr_Y:DoFOffset", "TransY", pxr.UsdPhysics.Tokens.transY),
    ("isaac:physics:Tr_Z:DoFOffset", "TransZ", pxr.UsdPhysics.Tokens.transZ),
    ("isaac:physics:Rot_X:DoFOffset", "RotX", pxr.UsdPhysics.Tokens.rotX),
    ("isaac:physics:Rot_Y:DoFOffset", "RotY", pxr.UsdPhysics.Tokens.rotY),
    ("isaac:physics:Rot_Z:DoFOffset", "RotZ", pxr.UsdPhysics.Tokens.rotZ),
)
_DOF_OFFSET_ATTR = "isaac:physics:DofOffsetOpOrder"
_TOKEN_FALLBACK_ORDER = {name: index for index, (_, name, _) in enumerate(_DEPRECATED_DOF_ATTRS)}


def _axis_has_valid_limits(joint_prim: pxr.Usd.Prim, axis_token: pxr.TfToken) -> bool:
    limit_api = pxr.UsdPhysics.LimitAPI.Get(joint_prim, axis_token)
    if not limit_api:
        return False
    lower_attr = limit_api.GetLowAttr()
    upper_attr = limit_api.GetHighAttr()
    if not lower_attr or not upper_attr:
        return False
    lower = lower_attr.Get()
    upper = upper_attr.Get()
    if lower is None or upper is None:
        return False
    try:
        return float(lower) < float(upper)
    except (TypeError, ValueError):
        return False


def _collect_deprecated_dof_entries(joint_prim: pxr.Usd.Prim) -> list[str]:
    entries: list[tuple[int, str]] = []
    for attr_name, token_name, axis_token in _DEPRECATED_DOF_ATTRS:
        attr = joint_prim.GetAttribute(attr_name)
        if not attr or not attr.HasAuthoredValueOpinion():
            continue
        if not _axis_has_valid_limits(joint_prim, axis_token):
            continue
        value = attr.Get()
        if value is None:
            continue
        try:
            order_index = int(value)
        except (TypeError, ValueError):
            continue
        entries.append((order_index, token_name))
    entries.sort(key=lambda item: (item[0], _TOKEN_FALLBACK_ORDER.get(item[1], item[0])))
    return [token for _, token in entries]


def _collect_robot_prims(
    stage: pxr.Usd.Stage,
    prim: pxr.Usd.Prim,
    *,
    target_api: str,
    relation: Relations,
    include_predicate: Callable[[pxr.Usd.Prim], bool] | None = None,
    descend_predicate: Callable[[pxr.Usd.Prim], bool] | None = None,
) -> list[pxr.Usd.Prim]:
    if not stage or not prim:
        return []

    include_predicate = include_predicate or (lambda node: node.HasAPI(target_api))
    descend_predicate = descend_predicate or (lambda _node: False)

    if include_predicate(prim):
        return [prim]
    if not prim.HasAPI(Classes.ROBOT_API.value):
        return []

    relationship = prim.GetRelationship(relation.name)
    if not relationship:
        return []

    collected: list[pxr.Usd.Prim] = []
    for target in relationship.GetTargets():
        child = stage.GetPrimAtPath(target)
        if not child:
            continue
        if include_predicate(child):
            collected.append(child)
            continue
        if descend_predicate(child):
            collected.extend(
                _collect_robot_prims(
                    stage,
                    child,
                    target_api=target_api,
                    relation=relation,
                    include_predicate=include_predicate,
                    descend_predicate=descend_predicate,
                )
            )
    return collected


def UpdateDeprecatedJointDofOrder(joint_prim: pxr.Usd.Prim) -> bool:
    """Update isaac:physics:DofOffsetOpOrder using legacy attributes when applicable."""
    if not joint_prim:
        return False
    if not (joint_prim.IsA(pxr.UsdPhysics.SphericalJoint) or joint_prim.IsA(pxr.UsdPhysics.D6Joint)):
        return False

    ordered_tokens = _collect_deprecated_dof_entries(joint_prim)
    if not ordered_tokens:
        return False

    dof_attr = joint_prim.GetAttribute(_DOF_OFFSET_ATTR)
    if dof_attr and dof_attr.HasAuthoredValueOpinion():
        current_value = dof_attr.Get()
        if current_value and list(current_value) == ordered_tokens:
            return False
    else:
        dof_attr = joint_prim.CreateAttribute(_DOF_OFFSET_ATTR, pxr.Sdf.ValueTypeNames.TokenArray, False)

    dof_attr.Set(pxr.Vt.TokenArray(ordered_tokens))
    return True


def GetAllRobotJoints(
    stage: pxr.Usd.Stage, robot_link_prim: pxr.Usd.Prim, parse_nested_robots: bool = True
) -> list[pxr.Usd.Prim]:
    """Returns all the joints of the robot.

    Args:
        stage: The USD stage containing the robot.
        robot_link_prim: The USD prim representing the robot link.
        parse_nested_robots: Whether to parse nested robots for joints.

    Returns:
        A list of USD prims representing the joints of the robot.
    """

    def _descend(child: pxr.Usd.Prim) -> bool:
        return parse_nested_robots and child.HasAPI(Classes.ROBOT_API.value)

    return _collect_robot_prims(
        stage,
        robot_link_prim,
        target_api=Classes.JOINT_API.value,
        relation=Relations.ROBOT_JOINTS,
        descend_predicate=_descend,
    )


def GetAllRobotLinks(
    stage: pxr.Usd.Stage, robot_link_prim: pxr.Usd.Prim, include_reference_points: bool = False
) -> list[pxr.Usd.Prim]:
    """Returns all the links of the robot.

    Args:
        stage: The USD stage containing the robot.
        robot_link_prim: The USD prim representing the robot link.
        include_reference_points: Whether to include reference points as links.

    Returns:
        A list of USD prims representing the links of the robot.
    """

    def _include(prim: pxr.Usd.Prim) -> bool:
        if prim.HasAPI(Classes.LINK_API.value):
            return True
        if include_reference_points and (
            prim.HasAPI(Classes.REFERENCE_POINT_API.value) or prim.HasAPI(Classes.SITE_API.value)
        ):
            if prim.HasAPI(Classes.REFERENCE_POINT_API.value):
                carb.log_warn(f"{prim.GetPath()} has ReferencePointAPI which is deprecated. Use SiteAPI instead.")
            return True
        return False

    return _collect_robot_prims(
        stage,
        robot_link_prim,
        target_api=Classes.LINK_API.value,
        relation=Relations.ROBOT_LINKS,
        include_predicate=_include,
        descend_predicate=lambda prim: prim.HasAPI(Classes.ROBOT_API.value),
    )


class RobotLinkNode:
    """Represents a node in the robot's kinematic tree structure.

    Attributes:
        prim: The USD prim representing this link
        path: The USD path to this link
        name: The name of this link
        _parent: Reference to the parent link node
        _children: List of child link nodes
        _joints: List of joints connected to this link
        _joint_to_parent: The joint connecting this link to its parent
    """

    def __init__(self, prim: pxr.Usd.Prim, parentLink: RobotLinkNode = None, joint: pxr.Usd.Prim = None):
        self.prim = prim
        if prim:
            self.path = prim.GetPath()
            self.name = prim.GetName()
        else:
            self.path = None
            self.name = None
        self._parent = parentLink
        self._children = []
        self._joints = []
        self._joint_to_parent = joint

    def add_child(self, child: RobotLinkNode):
        """Adds a child link node to this node.

        Args:
            child: The RobotLinkNode to add as a child
        """
        self.children.append(child)

    @property
    def children(self):
        """Returns the list of child nodes.

        Returns:
            list[RobotLinkNode]: List of child link nodes
        """
        return self._children

    @property
    def parent(self):
        """Returns the parent node.

        Returns:
            RobotLinkNode: The parent link node, or None if this is the root
        """
        return self._parent


def GetJointBodyRelationship(joint_prim: pxr.Usd.Prim, bodyIndex: int):
    """Gets the relationship target for a joint's body connection.

    Args:
        joint_prim: The USD prim representing the joint.
        bodyIndex: Index of the body (0 or 1) to get the relationship for.

    Returns:
        The target path of the body relationship, or None if not found or excluded from articulation.
    """
    joint = pxr.UsdPhysics.Joint(joint_prim)
    if joint:
        # Ignoring joints with ExcludeFromArticulation to avoid graph instances
        exclude_from_articulation = joint.GetExcludeFromArticulationAttr()
        if exclude_from_articulation and exclude_from_articulation.Get():
            return None
        rel = None
        if bodyIndex == 0:
            rel = joint.GetBody0Rel()
        elif bodyIndex == 1:
            rel = joint.GetBody1Rel()
        if rel:
            targets = rel.GetTargets()
            if targets:
                return targets[0]
    return None


def PrintRobotTree(root: RobotLinkNode, indent=0):
    """Prints a visual representation of the robot link tree structure.

    Args:
        root: The root node of the robot link tree.
        indent: Number of spaces to indent each level (default: 0).
    """
    print(" " * indent + root.name)
    for child in root.children:
        PrintRobotTree(child, indent + 2)


def _get_joint_local_transform(joint: pxr.UsdPhysics.Joint, body_index: int) -> pxr.Gf.Matrix4f | None:
    translate_attr = joint.GetLocalPos0Attr() if body_index == 0 else joint.GetLocalPos1Attr()
    rotate_attr = joint.GetLocalRot0Attr() if body_index == 0 else joint.GetLocalRot1Attr()
    if not translate_attr or not rotate_attr:
        return None
    translate = translate_attr.Get()
    rotate = rotate_attr.Get()
    if translate is None or rotate is None:
        return None
    local_transform = pxr.Gf.Matrix4f()
    local_transform.SetTranslate(translate)
    local_transform.SetRotateOnly(rotate)
    return local_transform


def GetJointPose(robot_prim: pxr.Usd.Prim, joint_prim: pxr.Usd.Prim) -> pxr.Gf.Matrix4d:
    """Returns the pose of a joint in the robot's coordinate system.

    Args:
        robot_prim: The USD prim representing the robot.
        joint_prim: The USD prim representing the joint.

    Returns:
        pxr.Gf.Matrix4d: The joint's pose matrix in robot coordinates, or None if pose cannot be computed.
    """
    robot_transform = pxr.Gf.Matrix4f(omni.usd.get_world_transform_matrix(robot_prim))
    joint = pxr.UsdPhysics.Joint(joint_prim)
    if not joint:
        return None

    stage = joint_prim.GetStage()
    inverse_robot = robot_transform.GetInverse()
    compose_order = {0: lambda local, body_pose: local * body_pose, 1: lambda local, body_pose: body_pose * local}

    for body_index in (0, 1):
        body_path = GetJointBodyRelationship(joint_prim, body_index)
        if not body_path:
            continue
        body_prim = stage.GetPrimAtPath(body_path)
        if not body_prim:
            continue
        local_transform = _get_joint_local_transform(joint, body_index)
        if not local_transform:
            continue
        body_pose = pxr.Gf.Matrix4f(omni.usd.get_world_transform_matrix(body_prim))
        joint_pose = compose_order[body_index](local_transform, body_pose)
        return joint_pose * inverse_robot

    return None


def GetLinksFromJoint(root: RobotLinkNode, joint_prim: pxr.Usd.Prim) -> tuple[list[pxr.Usd.Prim], list[pxr.Usd.Prim]]:
    """Returns two lists of links: those before and after the specified joint in the kinematic chain.

    Args:
        root: The root node of the robot link tree.
        joint_prim: The joint prim to find in the tree.

    Returns:
        tuple[list[pxr.Usd.Prim], list[pxr.Usd.Prim]]: Lists of links (before_joint, after_joint).
    """

    def find_node_with_joint(node: RobotLinkNode, target_joint: pxr.Usd.Prim) -> RobotLinkNode:
        """Helper function to find the node that has the target joint as _joint_to_parent."""
        if node._joint_to_parent == target_joint:
            return node
        for child in node.children:
            result = find_node_with_joint(child, target_joint)
            if result:
                return result
        return None

    def collect_forward_links(node: RobotLinkNode) -> list[pxr.Usd.Prim]:
        """Collects all links in the forward direction (children) from a node."""
        links = [node.prim]
        for child in node.children:
            links.extend(collect_forward_links(child))
        return links

    def collect_backward_links(node: RobotLinkNode) -> list[pxr.Usd.Prim]:
        """Collects all links in the backward direction (parents) from a node."""
        links = [node.prim]
        current = node.parent
        old_current = node
        while current:
            links.append(current.prim)
            if current:
                for child in current.children:
                    if child != old_current and child.prim not in links:
                        links += collect_forward_links(child)
            current = current.parent
        return links

    # Find the node that has our target joint as _joint_to_parent
    node_after_joint = find_node_with_joint(root, joint_prim)
    if not node_after_joint:
        return [], []

    # Get links after the joint (including the link after the joint)
    forward_links = collect_forward_links(node_after_joint)

    # Get links before the joint (including the link before the joint)
    backward_links = collect_backward_links(node_after_joint.parent)

    return backward_links, forward_links


def GenerateRobotLinkTree(stage: pxr.Usd.Stage, robot_link_prim: pxr.Usd.Prim = None) -> RobotLinkNode:
    """Generates a tree structure of robot links using an iterative approach.

    Args:
        stage: The USD stage containing the robot.
        robot_link_prim: The root prim of the robot.

    Returns:
        RobotLinkNode: The root node of the generated tree.
    """
    # Get all links and joints
    all_links = [a for a in pxr.Usd.PrimRange(robot_link_prim) if a.HasAPI(Classes.LINK_API.value)]
    all_joints = [a for a in pxr.Usd.PrimRange(robot_link_prim) if a.HasAPI(Classes.JOINT_API.value)]

    # Get root link and initialize mappings
    links = GetAllRobotLinks(stage, robot_link_prim)
    if not links:
        return None
    root = RobotLinkNode(links[0])
    joints_per_body = [{link.GetPath(): [] for link in all_links} for _ in range(2)]

    # Build joint mappings
    for joint in all_joints:
        for body_index in (0, 1):
            body = GetJointBodyRelationship(joint, body_index)
            if body and pxr.UsdPhysics.RigidBodyAPI(stage.GetPrimAtPath(body)):
                joints_per_body[body_index][body].append(joint)

    # Use a stack for iterative traversal
    stack = [root]
    processed_joints = []

    while stack:
        current = stack.pop()
        current_path = current.prim.GetPath()
        for i in range(2):
            for joint in joints_per_body[i].get(current_path, []):
                if joint not in processed_joints:
                    processed_joints.append(joint)
                    body1 = GetJointBodyRelationship(joint, 1)
                    # Check if it's not connecting to its parent
                    if body1 and (current.parent is None or current.parent.path != body1):
                        child = RobotLinkNode(stage.GetPrimAtPath(body1), current, joint)
                        current._joints.append(joint)
                        current.add_child(child)
                        stack.append(child)

    return root


def PopulateRobotSchemaFromArticulation(
    stage: pxr.Usd.Stage, robot_prim: pxr.Usd.Prim, articulation_prim: pxr.Usd.Prim | None = None
) -> tuple[pxr.Usd.Prim | None, pxr.Usd.Prim | None]:
    """Populate robot schema relationships using a PhysicsArticulation traversal.

    Discovers the articulation root link and root joint, walks all connected rigid bodies through
    their joints, applies the LinkAPI and JointAPI schemas, and updates the robot relationships
    with the ordered lists of discovered prims.

    Args:
        stage: Stage containing the robot articulation.
        robot_prim: Prim that already has the RobotAPI applied.
        articulation_prim: Optional prim that owns the PhysicsArticulationRootAPI. Defaults
            to ``robot_prim`` when omitted.

    Returns:
        tuple[pxr.Usd.Prim | None, pxr.Usd.Prim | None]: The detected root link prim and root joint prim.

    Raises:
        ValueError: If the stage or robot prim is invalid.

    Example:

    .. code-block:: python

        >>> stage = omni.usd.get_context().get_stage()
        >>> robot = stage.GetPrimAtPath("/World/MyRobot")
        >>> root_link, root_joint = PopulateRobotSchemaFromArticulation(stage, robot, robot)
    """
    if not stage:
        raise ValueError("Stage is invalid.")
    if not robot_prim:
        raise ValueError("Robot prim is invalid.")

    articulation_root = articulation_prim or robot_prim
    if not articulation_root:
        return None, None

    if not articulation_root.HasAPI(pxr.UsdPhysics.ArticulationRootAPI):
        for prim in pxr.Usd.PrimRange(articulation_root):
            if prim.HasAPI(pxr.UsdPhysics.ArticulationRootAPI):
                articulation_root = prim
                break
        else:
            return None, None

    root_link = articulation_root
    articulation_joints = [prim for prim in pxr.Usd.PrimRange(robot_prim) if prim.IsA(pxr.UsdPhysics.Joint)]

    def _path_key(path: pxr.Sdf.Path | None) -> str | None:
        if not path:
            return None
        return str(path)

    root_joint = None
    if articulation_root.IsA(pxr.UsdPhysics.Joint):
        root_joint = articulation_root
        candidate_path = GetJointBodyRelationship(root_joint, 0) or GetJointBodyRelationship(root_joint, 1)
        if candidate_path:
            root_link = stage.GetPrimAtPath(candidate_path)
    if not root_link:
        return None, root_joint
    root_link_key = str(root_link.GetPath())
    body_to_joints: dict[str, list[tuple[pxr.Usd.Prim, int]]] = {}
    for joint_prim in articulation_joints:
        for body_index in (0, 1):
            body_path = GetJointBodyRelationship(joint_prim, body_index)
            key = _path_key(body_path)
            if not key:
                continue
            body_to_joints.setdefault(key, []).append((joint_prim, body_index))
            if not root_joint and root_link_key and key == root_link_key:
                root_joint = joint_prim

    from . import ApplyJointAPI, ApplyLinkAPI

    queue: deque[pxr.Usd.Prim] = deque()
    visited_links: set[str] = set()
    visited_joints: set[str] = set()
    ordered_links: list[pxr.Usd.Prim] = []
    ordered_joints: list[pxr.Usd.Prim] = []

    if root_joint:
        ordered_joints.append(root_joint)
        visited_joints.add(str(root_joint.GetPath()))
        ApplyJointAPI(root_joint)

    if root_link:
        queue.append(root_link)

    while queue:
        link_prim = queue.popleft()
        if not link_prim:
            continue
        link_key = str(link_prim.GetPath())
        if link_key in visited_links:
            continue
        visited_links.add(link_key)
        ApplyLinkAPI(link_prim)
        ordered_links.append(link_prim)

        for joint_prim, body_index in body_to_joints.get(link_key, []):
            joint_key = str(joint_prim.GetPath())
            if joint_key not in visited_joints:
                ApplyJointAPI(joint_prim)
                ordered_joints.append(joint_prim)
                visited_joints.add(joint_key)

            other_index = 1 - body_index
            other_path = GetJointBodyRelationship(joint_prim, other_index)
            if not other_path:
                continue
            other_key = str(other_path)
            if other_key in visited_links:
                continue
            other_prim = stage.GetPrimAtPath(other_path)
            if other_prim:
                queue.append(other_prim)

    robot_links_rel = robot_prim.CreateRelationship(Relations.ROBOT_LINKS.name, custom=True)
    for prim in ordered_links:
        if prim.HasAPI(Classes.LINK_API.value):
            robot_links_rel.AddTarget(prim.GetPath())

    robot_joints_rel = robot_prim.CreateRelationship(Relations.ROBOT_JOINTS.name, custom=True)
    for prim in ordered_joints:
        if prim.HasAPI(Classes.JOINT_API.value):
            robot_joints_rel.AddTarget(prim.GetPath())

    return root_link, root_joint


def UpdateDeprecatedSchemas(robot_prim: pxr.Usd.Prim):
    if not robot_prim or not robot_prim.IsValid():
        return

    for prim in pxr.Usd.PrimRange(robot_prim):
        if prim.HasAPI(Classes.REFERENCE_POINT_API.value):
            prim.RemoveAppliedSchema(Classes.REFERENCE_POINT_API.value)
            prim.AddAppliedSchema(Classes.SITE_API.value)
        if prim.HasAPI(Classes.JOINT_API.value):
            UpdateDeprecatedJointDofOrder(prim)
