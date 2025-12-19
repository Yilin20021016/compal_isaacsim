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

import asyncio

import carb
import omni
from isaacsim.core.experimental.utils import stage as stage_utils
from isaacsim.core.simulation_manager import SimulationManager

try:
    from isaacsim.test.utils import TimedAsyncTestCase
except ImportError:
    raise ImportError(
        "isaacsim.test.utils is required to use ROS2TestCase. " "This module should only be imported in test contexts."
    )


class ROS2TestCase(TimedAsyncTestCase):
    """Base test class that automatically times all test methods.

    This class extends TimedAsyncTestCase to add ROS2 specific setup and teardown.
    It also provides helper methods for creating and managing ROS2 resources with automatic cleanup.
    """

    async def setUp(self):
        """Set up test timing before each test method."""
        await super().setUp()
        self._timeline = omni.timeline.get_timeline_interface()

        # Initialize lists to track ROS2 resources for automatic cleanup
        self._ros2_nodes = []
        self._ros2_publishers = []
        self._ros2_subscribers = []

        import rclpy

        if not rclpy.ok():
            rclpy.init()

        SimulationManager.set_backend("numpy")
        SimulationManager.set_physics_sim_device("cpu")
        SimulationManager.enable_fabric(enable=False)
        await omni.kit.app.get_app().next_update_async()
        await stage_utils.create_new_stage_async()
        try:
            from isaacsim.core.rendering_manager import ViewportManager

            status, frames = await ViewportManager.wait_for_viewport_async()
            self.assertTrue(status, f"Viewport not ready after {frames} frames")
        except:
            pass

    def create_node(self, node_name):
        """Create a ROS2 node and track it for automatic cleanup.

        Args:
            node_name: Name of the ROS2 node to create.

        Returns:
            The created ROS2 node.
        """
        import rclpy

        node = rclpy.create_node(node_name)
        self._ros2_nodes.append(node)
        return node

    def create_publisher(self, node, msg_type, topic_name, qos_profile=10):
        """Create a ROS2 publisher and track it for automatic cleanup.

        Args:
            node: The ROS2 node to create the publisher on.
            msg_type: The message type for the publisher.
            topic_name: The topic name to publish to.
            qos_profile: QoS profile for the publisher (default: 10).

        Returns:
            The created ROS2 publisher.
        """
        publisher = node.create_publisher(msg_type, topic_name, qos_profile)
        self._ros2_publishers.append((node, publisher))
        return publisher

    def create_subscription(self, node, msg_type, topic_name, callback, qos_profile=10):
        """Create a ROS2 subscription and track it for automatic cleanup.

        Args:
            node: The ROS2 node to create the subscription on.
            msg_type: The message type for the subscription.
            topic_name: The topic name to subscribe to.
            callback: Callback function for received messages.
            qos_profile: QoS profile for the subscription (default: 10).

        Returns:
            The created ROS2 subscription.
        """
        subscription = node.create_subscription(msg_type, topic_name, callback, qos_profile)
        self._ros2_subscribers.append((node, subscription))
        return subscription

    def destroy_subscription(self, node, subscription):
        """Manually destroy a subscription and remove it from tracking.

        Args:
            node: The ROS2 node that owns the subscription.
            subscription: The subscription to destroy.
        """
        try:
            node.destroy_subscription(subscription)
            # Remove from tracking list to avoid double cleanup
            if (node, subscription) in self._ros2_subscribers:
                self._ros2_subscribers.remove((node, subscription))
        except Exception as e:
            print(f"Warning: Failed to destroy subscription: {e}")

    def destroy_publisher(self, node, publisher):
        """Manually destroy a publisher and remove it from tracking.

        Args:
            node: The ROS2 node that owns the publisher.
            publisher: The publisher to destroy.
        """
        try:
            node.destroy_publisher(publisher)
            # Remove from tracking list to avoid double cleanup
            if (node, publisher) in self._ros2_publishers:
                self._ros2_publishers.remove((node, publisher))
        except Exception as e:
            print(f"Warning: Failed to destroy publisher: {e}")

    async def tearDown(self):
        self._timeline.stop()
        await omni.kit.app.get_app().next_update_async()
        while omni.usd.get_context().get_stage_loading_status()[2] > 0:
            print("tearDown, assets still loading, waiting to finish...")
            await asyncio.sleep(1.0)

        # Clean up ROS2 resources in the correct order
        # First destroy publishers
        for node, publisher in self._ros2_publishers:
            try:
                node.destroy_publisher(publisher)
            except Exception as e:
                print(f"Warning: Failed to destroy publisher: {e}")

        # Then destroy subscribers
        for node, subscription in self._ros2_subscribers:
            try:
                node.destroy_subscription(subscription)
            except Exception as e:
                print(f"Warning: Failed to destroy subscription: {e}")

        # Finally destroy nodes
        for node in self._ros2_nodes:
            try:
                node.destroy_node()
            except Exception as e:
                print(f"Warning: Failed to destroy node: {e}")

        # Clear the tracking lists
        self._ros2_publishers.clear()
        self._ros2_subscribers.clear()
        self._ros2_nodes.clear()

        import rclpy

        if rclpy.ok():
            rclpy.shutdown()

        await super().tearDown()
