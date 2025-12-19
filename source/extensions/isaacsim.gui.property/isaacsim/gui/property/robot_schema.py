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

import carb
import omni
import omni.ui as ui
from omni.kit.property.usd.prim_selection_payload import PrimSelectionPayload
from omni.kit.property.usd.usd_property_widget import UsdPropertiesWidget
from pxr import Sdf, Usd
from usd.schema.isaac import robot_schema

_EXCLUSIVE_SCHEMAS = (
    robot_schema.Classes.ROBOT_API,
    robot_schema.Classes.LINK_API,
    robot_schema.Classes.JOINT_API,
)


def Singleton(class_):
    """A singleton decorator"""
    instances = {}

    def getinstance(*args, **kwargs):
        if class_ not in instances:
            instances[class_] = class_(*args, **kwargs)
        return instances[class_]

    return getinstance


class _RobotSchemaWidgetBase(UsdPropertiesWidget):
    _MENU_PREFIX = "Isaac/Robot Schema"

    def __init__(
        self,
        title,
        collapsed,
        schema_class,
        attributes,
        menu_label,
        apply_fn,
        relationships=None,
        exclusive_classes=None,
    ):
        super().__init__(title, collapsed)
        from omni.kit.property.usd import PrimPathWidget

        self._schema_class = schema_class
        self._attributes = attributes
        self._relationships = relationships or []
        self._apply_fn = apply_fn
        self._menu_label = menu_label
        self._attr_map = {attr.name: attr for attr in attributes}
        self._relationship_map = {rel.name: rel for rel in self._relationships}
        self._prim = None
        self._old_payload = None
        self._exclusive_classes = exclusive_classes or _EXCLUSIVE_SCHEMAS

        self._menu_entries = [
            PrimPathWidget.add_button_menu_entry(
                f"{self._MENU_PREFIX}/{menu_label}", show_fn=self._button_show, onclick_fn=self._button_onclick
            )
        ]

    def destroy(self):
        from omni.kit.property.usd import PrimPathWidget

        for menu in self._menu_entries:
            PrimPathWidget.remove_button_menu_entry(menu)
        self._menu_entries = []

    def _button_show(self, objects: dict):
        stage = objects.get("stage")
        prim_list = objects.get("prim_list")
        if not stage or not prim_list:
            return False
        for item in prim_list:
            prim = stage.GetPrimAtPath(item) if isinstance(item, Sdf.Path) else item
            if prim and not self._has_exclusive_schema(prim):
                return True
        return False

    def _button_onclick(self, payload: PrimSelectionPayload):
        stage = self._payload.get_stage() if self._payload else omni.usd.get_context().get_stage()
        if not stage:
            return

        for path in payload:
            if not path:
                continue
            prim = stage.GetPrimAtPath(path)
            if not prim or self._has_exclusive_schema(prim):
                continue
            self._apply_fn(prim)

            instanceable = [p for p in Usd.PrimRange(prim) if p.IsInstanceable()]
            if instanceable:
                prim.SetInstanceable(True)
                prim.ClearMetadata("instanceable")
        self._request_refresh()

    def _request_refresh(self):
        """Refresh the property window without touching USD selection."""
        property_window = omni.kit.window.property.get_window()
        if not property_window or not property_window._window:  # noqa: SLF001
            return
        property_window._window.frame.rebuild()  # noqa: SLF001

    def _on_usd_changed(self, notice, stage):
        targets = notice.GetChangedInfoOnlyPaths()
        if self._old_payload != self.on_new_payload(self._payload):
            self._request_refresh()
        else:
            super()._on_usd_changed(notice, stage)

    def _get_prim(self, prim_path):
        if prim_path:
            stage = self._payload.get_stage()
            if stage:
                prim = stage.GetPrimAtPath(prim_path)
                if prim and prim.HasAPI(self._schema_class.value):
                    return prim
        return None

    def on_new_payload(self, payload):
        """
        See PropertyWidget.on_new_payload
        """

        if not super().on_new_payload(payload):
            return False

        if len(self._payload) != 1:
            return False
        prim_path = self._payload.get_paths()[0]
        self._prim = self._get_prim(prim_path)
        self._old_payload = self._prim
        if not self._prim:
            return False

        return self._prim

    def on_remove_attr(self):
        stage = self._payload.get_stage()
        if not stage or not self._payload:
            return

        prim = self._get_prim(self._payload.get_paths()[0])
        if not prim:
            return

        if prim.HasAPI(self._schema_class.value):
            prim.RemoveAppliedSchema(self._schema_class.value)
        for attr in self._attributes:
            if prim.HasAttribute(attr.name):
                prim.RemoveProperty(attr.name)
        for rel in self._relationships:
            if prim.HasRelationship(rel.name):
                prim.RemoveProperty(rel.name)
        self._request_refresh()

    def _filter_props_to_build(self, props):
        filtered = []
        for prop in props:
            if isinstance(prop, Usd.Attribute) and prop.GetName() in self._attr_map:
                attr = self._attr_map[prop.GetName()]
                prop.SetDisplayName(attr.display_name)
                filtered.append(prop)
            elif isinstance(prop, Usd.Relationship) and prop.GetName() in self._relationship_map:
                relationship = self._relationship_map[prop.GetName()]
                prop.SetDisplayName(relationship.display_name)
                filtered.append(prop)
        return filtered

    def _has_exclusive_schema(self, prim):
        for schema in self._exclusive_classes:
            if prim.HasAPI(schema.value):
                return True
        return False

    def build_items(self):
        if self._collapsable_frame and not self._collapsable_frame.collapsed and self._prim:
            super().build_items()

    def _build_frame_header(self, collapsed, text: str, id: str = None):
        """Custom header for CollapsableFrame"""
        if id is None:
            id = text

        if collapsed:
            alignment = ui.Alignment.RIGHT_CENTER
            width = 5
            height = 7
        else:
            alignment = ui.Alignment.CENTER_BOTTOM
            width = 7
            height = 5

        header_stack = ui.HStack(spacing=8)
        with header_stack:
            with ui.VStack(width=0):
                ui.Spacer()
                ui.Triangle(
                    style_type_name_override="CollapsableFrame.Header", width=width, height=height, alignment=alignment
                )
                ui.Spacer()
            ui.Label(text, style_type_name_override="CollapsableFrame.Header", width=ui.Fraction(1))

            button_style = {"Button.Image": {"color": 0xFFFFFFFF, "alignment": ui.Alignment.CENTER}}

            ui.Spacer(width=ui.Fraction(6))
            button_style["Button.Image"]["image_url"] = "${icons}/Cancel_64.png"

            with ui.ZStack(content_clipping=True, width=16, height=16):
                ui.Button(
                    "",
                    style=button_style,
                    clicked_fn=self.on_remove_attr,
                    identifier=f"remove_{self._schema_class.value}",
                    tooltip=f"Remove {self._schema_class.value}",
                )


@Singleton
class RobotAPIWidget(_RobotSchemaWidgetBase):
    def __init__(self, title: str, collapsed: bool = False):
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.ROBOT_API,
            [
                robot_schema.Attributes.DESCRIPTION,
                robot_schema.Attributes.NAMESPACE,
                robot_schema.Attributes.ROBOT_TYPE,
                robot_schema.Attributes.LICENSE,
                robot_schema.Attributes.VERSION,
                robot_schema.Attributes.SOURCE,
                robot_schema.Attributes.CHANGELOG,
            ],
            "Robot API",
            robot_schema.ApplyRobotAPI,
            relationships=[
                robot_schema.Relations.ROBOT_LINKS,
                robot_schema.Relations.ROBOT_JOINTS,
            ],
        )


@Singleton
class LinkAPIWidget(_RobotSchemaWidgetBase):
    def __init__(self, title: str, collapsed: bool = False):
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.LINK_API,
            [
                robot_schema.Attributes.NAME_OVERRIDE,
            ],
            "Link API",
            robot_schema.ApplyLinkAPI,
        )


@Singleton
class JointAPIWidget(_RobotSchemaWidgetBase):
    def __init__(self, title: str, collapsed: bool = False):
        super().__init__(
            title,
            collapsed,
            robot_schema.Classes.JOINT_API,
            [
                robot_schema.Attributes.JOINT_NAME_OVERRIDE,
                robot_schema.Attributes.DOF_OFFSET_OP_ORDER,
                robot_schema.Attributes.ACTUATOR,
            ],
            "Joint API",
            robot_schema.ApplyJointAPI,
        )
