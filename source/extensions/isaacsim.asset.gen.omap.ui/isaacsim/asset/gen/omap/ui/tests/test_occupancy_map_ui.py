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
import omni.kit.test
import omni.kit.ui_test as ui_test


class TestOccupancyMapUI(omni.kit.test.AsyncTestCase):
    """Test suite for the Occupancy Map UI extension.

    Tests the UI functionality including menu loading and basic interaction with
    the occupancy map generation interface.
    """

    async def setUp(self):
        """Sets up the test environment.

        Preloads materials and waits for UI to stabilize before running tests.
        """
        await ui_test.human_delay()

    async def tearDown(self):
        """Cleans up after each test."""
        pass

    async def testLoading(self):
        """Tests that the Occupancy Map UI can be loaded from the menu.

        Creates a new stage and navigates through the Tools > Robotics > Occupancy Map
        menu to verify the extension loads correctly.
        """
        await omni.usd.get_context().new_stage_async()
        menu_widget = ui_test.get_menubar()
        await menu_widget.find_menu("Tools").click()
        await menu_widget.find_menu("Robotics").click()
        await menu_widget.find_menu("Occupancy Map").click()
