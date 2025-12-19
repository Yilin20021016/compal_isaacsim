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


import os
import subprocess
import sys

import carb
import omni.ext

from .ros2_common import SUPPORTED_ROS_DISTROS, get_ubuntu_version, print_environment_setup_instructions


class ROS2CoreExtension(omni.ext.IExt):
    """ROS 2 Core Extension - Foundation extension for ROS 2 integration.

    This extension provides the core C++ plugin, factory, context management,
    and foundational utilities for ROS 2 integration in Isaac Sim.
    """

    def on_startup(self, ext_id):
        """Initialize the ROS 2 core foundation extension.

        Args:
            ext_id: The extension ID.
        """
        self._ros2bridge = None
        self._module = None
        self._rclpy_instance = None

        ext_manager = omni.kit.app.get_app().get_extension_manager()
        self._extension_path = ext_manager.get_extension_path(ext_id)

        backup_ros_distro = carb.settings.get_settings().get_as_string("/exts/isaacsim.ros2.bridge/ros_distro")
        ros_distro = os.environ.get("ROS_DISTRO")
        if ros_distro is None:

            if backup_ros_distro == "system_default":
                if sys.platform == "linux":
                    detected_distro = get_ubuntu_version()
                    if detected_distro:
                        backup_ros_distro = detected_distro
                        carb.log_info(
                            f"Ubuntu distro detected. Using system default ROS distribution: {backup_ros_distro}"
                        )
                    else:
                        backup_ros_distro = "humble"
                        carb.log_error(f"Unsupported Ubuntu distro. Setting ROS distribution to: {backup_ros_distro}")
                else:
                    # For Windows system, default to humble
                    backup_ros_distro = "humble"
                    carb.log_warn("Using 'humble' as default")

            omni.kit.app.get_app().print_and_log(f"Using backup internal ROS2 {backup_ros_distro} distro")
            ros_distro = backup_ros_distro
            os.environ["ROS_DISTRO"] = ros_distro

        if ros_distro not in SUPPORTED_ROS_DISTROS.values():
            carb.log_error(f"ROS_DISTRO of {ros_distro} is currently not supported")
            ext_manager.set_extension_enabled("isaacsim.ros2.core", False)
            return

        if sys.platform == "win32":
            if os.environ.get("PATH"):
                os.environ["PATH"] = os.environ.get("PATH") + ";" + self._extension_path + "/bin"
                # WAR: sys.path on windows is missing PYTHONPATH variables, causing rclpy to not be found
                if os.environ.get("PYTHONPATH") is not None:
                    sys.path.extend(os.environ.get("PYTHONPATH").split(";"))
            else:
                os.environ["PATH"] = self._extension_path + "/bin"

        carb.get_framework().load_plugins(
            loaded_file_wildcards=["isaacsim.ros2.core.plugin"],
            search_paths=[os.path.abspath(os.path.join(self._extension_path, "bin"))],
        )
        from isaacsim.ros2.core.bindings import _ros2_core

        self._module = _ros2_core

        if self.check_status(os.environ["ROS_DISTRO"]) is False:
            print_environment_setup_instructions(self._extension_path, ros_distro)
            carb.log_error(f"ROS2 Bridge startup failed")
            ext_manager.set_extension_enabled("isaacsim.ros2.core", False)
        else:
            self._ros2bridge = self._module.acquire_ros2_core_interface()
            if self._ros2bridge.get_startup_status() is False:
                carb.log_error(f"ROS2 Bridge startup failed")
                return

            # manually load the rclpy extension, only if we know ROS 2 is working
            if f"{self._extension_path}" not in sys.path:
                sys.path.append(f"{self._extension_path}")

            if ros_distro == "humble":
                from humble.rclpy import Extension as rclpy_ext

                self._rclpy_instance = rclpy_ext()
            elif ros_distro == "jazzy":
                from jazzy.rclpy import Extension as rclpy_ext

                self._rclpy_instance = rclpy_ext()
            else:
                self._rclpy_instance = None

            if self._rclpy_instance is not None:
                self._rclpy_instance.on_startup("isaacsim.ros2.core")

            carb.log_info("ROS2 Core: Foundation extension loaded successfully")

    def on_shutdown(self):
        """Shutdown the core extension.

        Properly shutdown the rclpy extension and core interface.
        """
        if hasattr(self, "_ros2bridge") and self._ros2bridge is not None:
            if hasattr(self, "_module") and self._module is not None:
                self._module.release_ros2_core_interface(self._ros2bridge)

        if hasattr(self, "_rclpy_instance") and self._rclpy_instance is not None:
            self._rclpy_instance.on_shutdown()

        carb.log_info("ROS2 Core: Foundation extension shutdown complete")

    def check_status(self, distro):
        # Run an external process that checks if ROS2 can be loaded
        # If ROS2 cannot be loaded a memory leak occurs, running in a separate process prevent this
        path = os.path.abspath(self._extension_path + "/bin")
        ros_lib_path = os.path.join(os.path.abspath(f"{self._extension_path}/{distro}/lib"), "")

        command = [f'./isaacsim.ros2.core.check "{ros_lib_path}"']
        if sys.platform == "win32":

            command = [f"isaacsim.ros2.core.check.exe", f"{ros_lib_path}"]

        try:
            output = subprocess.check_output(command, shell=True, cwd=f"{path}")
            return True
        except subprocess.CalledProcessError as grepexc:
            print(grepexc.output.decode("utf-8"))
            return False
