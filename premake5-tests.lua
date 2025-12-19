-- SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
-- SPDX-License-Identifier: Apache-2.0
--
-- Licensed under the Apache License, Version 2.0 (the "License");
-- you may not use this file except in compliance with the License.
-- You may obtain a copy of the License at
--
-- http://www.apache.org/licenses/LICENSE-2.0
--
-- Unless required by applicable law or agreed to in writing, software
-- distributed under the License is distributed on an "AS IS" BASIS,
-- WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
-- See the License for the specific language governing permissions and
-- limitations under the License.

-- Function to define test startup experience with app name, kit file, and optional extra arguments
function define_test_startup_experience(app_name, kit_file, extra_args)
    local script_dir_token = (os.target() == "windows") and "%~dp0" or "$SCRIPT_DIR"
    local extra_args = extra_args or ""
    local kit_file = kit_file or app_name
    define_test_experience(app_name, {
        config_path = "../apps/" .. kit_file .. ".kit",
        extra_args = '--ext-folder "' .. script_dir_token .. '/../exts" ' ..
                     '--ext-folder "' .. script_dir_token .. '/../extscache" ' ..
                     '--ext-folder "' .. script_dir_token .. '/../extsDeprecated" ' ..
                     '--ext-folder "' .. script_dir_token .. '/../apps" ' .. extra_args,
    })
end

-- Helper function to define a group of related tests
function define_test_group(group_name, tests)
    group(group_name)
    for _, test in ipairs(tests) do
        define_test_startup_experience(test.name, test.kit_file, test.extra_args)
    end
end

-- Helper to register a list of python sample tests
local function register_python_sample_tests(tests)
    for _, test in ipairs(tests) do
        python_sample_test(test[1], test[2], test[3])
    end
end

local function get_startup_tests()
    return {
        {
            name = "tests-startup.main",
            kit_file = "isaacsim.exp.full",
            extra_args = "--/app/quitAfter=100 --/app/file/ignoreUnsavedOnExit=1",
        },
        {
            name = "tests-startup.streaming",
            kit_file = "isaacsim.exp.full.streaming",
            extra_args = "--no-window --/app/quitAfter=100 --/app/file/ignoreUnsavedOnExit=1",
        },
        {
            name = "tests-startup.extscache",
            kit_file = "isaacsim.exp.full",
            extra_args = "--no-window --/app/quitAfter=100 --/app/extensions/registryEnabled=0 --/app/file/ignoreUnsavedOnExit=1",
        },
        {
            name = "tests-startup.xr.vr",
            kit_file = "isaacsim.exp.base.xr.vr",
            extra_args = "--no-window --/app/quitAfter=100 --/app/file/ignoreUnsavedOnExit=1",
        },
    }
end

local function get_simulation_app_tests()
    return {
        {
            "tests-nativepython-isaacsim.simulation_app.hello_world",
            "standalone_examples/api/isaacsim.simulation_app/hello_world.py",
        },
        {
            "tests-nativepython-isaacsim.simulation_app.change_resolution",
            "standalone_examples/api/isaacsim.simulation_app/change_resolution.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.simulation_app.load_stage",
            "standalone_examples/api/isaacsim.simulation_app/load_stage.py",
            "--usd_path /Isaac/Environments/Simple_Room/simple_room.usd --test --headless",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_config",
            "standalone_examples/testing/isaacsim.simulation_app/test_config.py",
            "--/persistent/isaac/asset_root/default=omniverse://ov-test-this-is-working",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_multiprocess",
            "standalone_examples/testing/isaacsim.simulation_app/test_multiprocess.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_viewport_ready",
            "standalone_examples/testing/isaacsim.simulation_app/test_viewport_ready.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_headless_no_rendering",
            "standalone_examples/testing/isaacsim.simulation_app/test_headless_no_rendering.py",
        },
        -- Additional simulation app tests
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_frame_delay_basic",
            "standalone_examples/testing/isaacsim.simulation_app/test_frame_delay.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_fabric_frame_delay",
            "standalone_examples/testing/isaacsim.simulation_app/test_fabric_frame_delay.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_ogn",
            "standalone_examples/testing/isaacsim.simulation_app/test_ogn.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_syntheticdata",
            "standalone_examples/testing/isaacsim.simulation_app/test_syntheticdata.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_fetch_results",
            "standalone_examples/testing/isaacsim.simulation_app/test_fetch_results.py",
        },
        {
            "tests-nativepython-testing-isaacsim.benchmark.services.test_no_rendering",
            "standalone_examples/testing/isaacsim.benchmark.services/test_no_rendering.py",
        },
        {
            "tests-nativepython-testing-isaacsim.simulation_app.test_external",
            "standalone_examples/testing/isaacsim.simulation_app/test_external.py",
            "--enable omni.kit.scripting",
        },
    }
end

local function get_core_tests()
    return {
        -- Cloner
        {
            "tests-nativepython-isaacsim.core.cloner.clone_ants",
            "standalone_examples/api/isaacsim.core.cloner/clone_ants.py",
        },
        -- Core API
        { "tests-nativepython-isaacsim.core.api.add_cubes", "standalone_examples/api/isaacsim.core.api/add_cubes.py" },
        {
            "tests-nativepython-isaacsim.core.api.add_frankas",
            "standalone_examples/api/isaacsim.core.api/add_frankas.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.api.data_logging",
            "standalone_examples/api/isaacsim.core.api/data_logging.py",
        },
        {
            "tests-nativepython-isaacsim.core.api.control_robot",
            "standalone_examples/api/isaacsim.core.api/control_robot.py",
        },
        {
            "tests-nativepython-isaacsim.core.api.simulate_robot",
            "standalone_examples/api/isaacsim.core.api/simulate_robot.py",
        },
        {
            "tests-nativepython-isaacsim.core.api.simulation_callbacks",
            "standalone_examples/api/isaacsim.core.api/simulation_callbacks.py",
        },
        {
            "tests-nativepython-isaacsim.core.api.time_stepping",
            "standalone_examples/api/isaacsim.core.api/time_stepping.py",
        },
        {
            "tests-nativepython-isaacsim.core.api.visual_materials",
            "standalone_examples/api/isaacsim.core.api/visual_materials.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.api.omnigraph_triggers",
            "standalone_examples/api/isaacsim.core.api/omnigraph_triggers.py",
        },
        {
            "tests-nativepython-isaacsim.core.api.cloth",
            "standalone_examples/api/isaacsim.core.api/cloth.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.api.rigid_contact_view",
            "standalone_examples/api/isaacsim.core.api/rigid_contact_view.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.core.api.detailed_contact_data",
            "standalone_examples/api/isaacsim.core.api/detailed_contact_data.py",
            "--test",
        },
        -- From Misc
        {
            "tests-nativepython-testing-omni.syntheticdata.test_basic",
            "standalone_examples/testing/omni.syntheticdata/test_basic.py",
        },
        -- Cortex
        {
            "tests-nativepython-testing-isaacsim.cortex.framework.bringup",
            "standalone_examples/testing/isaacsim.cortex.framework/cortex_bringup_test.py",
        },
    }
end

local function get_sensor_tests()
    return {
        -- Camera
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_add_depth_sensor",
            "standalone_examples/api/isaacsim.sensors.camera/camera_add_depth_sensor.py",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_opencv_fisheye",
            "standalone_examples/api/isaacsim.sensors.camera/camera_opencv_fisheye.py",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_opencv_pinhole",
            "standalone_examples/api/isaacsim.sensors.camera/camera_opencv_pinhole.py",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_pre_isp_pipeline",
            "standalone_examples/api/isaacsim.sensors.camera/camera_pre_isp_pipeline.py",
            "--draw-output",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_ros",
            "standalone_examples/api/isaacsim.sensors.camera/camera_ros.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_view",
            "standalone_examples/api/isaacsim.sensors.camera/camera_view.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera",
            "standalone_examples/api/isaacsim.sensors.camera/camera.py",
            "--test --disable_output",
        },
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_stereoscopic_depth",
            "standalone_examples/api/isaacsim.sensors.camera/camera_stereoscopic_depth.py",
            "--test",
        },
        -- From Misc Camera
        {
            "tests-nativepython-isaacsim.sensors.camera.camera_annotator_device",
            "standalone_examples/api/isaacsim.sensors.camera/camera_annotator_device.py",
        },
        -- RTX
        {
            "tests-nativepython-isaacsim.sensors.rtx.inspect_lidar_metadata",
            "standalone_examples/api/isaacsim.sensors.rtx/inspect_lidar_metadata.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.rtx.inspect_radar_metadata",
            "standalone_examples/api/isaacsim.sensors.rtx/inspect_radar_metadata.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.rtx.resolve_object_ids_from_gmo",
            "standalone_examples/api/isaacsim.sensors.rtx/resolve_object_ids_from_gmo.py",
        },
        {
            "tests-nativepython-isaacsim.sensors.rtx.rotating_lidar_rtx",
            "standalone_examples/api/isaacsim.sensors.rtx/rotating_lidar_rtx.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.sensors.rtx.specify_non_visual_materials",
            "standalone_examples/api/isaacsim.sensors.rtx/specify_non_visual_materials.py",
            "--test",
        },
        -- Physics
        {
            "tests-nativepython-isaacsim.sensors.physx.rotating_lidar_physX",
            "standalone_examples/api/isaacsim.sensors.physx/rotating_lidar_physX.py",
            "--test",
        },
        -- From Misc Physics
        {
            "tests-nativepython-testing-isaacsim.sensors.physics.contact_sensor",
            "standalone_examples/testing/isaacsim.sensors.physics/contact_sensor_test.py",
        },
    }
end

local function get_robot_tests()
    return {
        -- Manipulators
        {
            "tests-nativepython-isaacsim.robot.manipulators.franka.franka_gripper",
            "standalone_examples/api/isaacsim.robot.manipulators/franka/franka_gripper.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.manipulators.cobotta_900.follow_target_example",
            "standalone_examples/api/isaacsim.robot.manipulators/cobotta_900/follow_target_example.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.manipulators.cobotta_900.pick_up_example",
            "standalone_examples/api/isaacsim.robot.manipulators/cobotta_900/pick_up_example.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.manipulators.cobotta_900.gripper_control",
            "standalone_examples/api/isaacsim.robot.manipulators/cobotta_900/gripper_control.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.manipulators.franka_pick_up",
            "standalone_examples/api/isaacsim.robot.manipulators/franka_pick_up.py",
            "--test",
        },
        {
            "tests-nativepython-isaacsim.robot.manipulators.ur10_pick_up",
            "standalone_examples/api/isaacsim.robot.manipulators/ur10_pick_up.py",
            "--test",
        },
        -- Wheeled Robots
        {
            "tests-nativepython-isaacsim.robot.wheeled_robots.examples.jetbot_differential_move",
            "standalone_examples/api/isaacsim.robot.wheeled_robots.examples/jetbot_differential_move.py",
            "--test",
        },
    }
end

local function get_asset_tests()
    return {
        -- URDF
        {
            "tests-nativepython-isaacsim.asset.importer.urdf.urdf_import",
            "standalone_examples/api/isaacsim.asset.importer.urdf/urdf_import.py",
        },
        -- MJCF
        {
            "tests-nativepython-isaacsim.asset.importer.mjcf.mjcf_import",
            "standalone_examples/api/isaacsim.asset.importer.mjcf/mjcf_import.py",
        },
    }
end

local function get_replicator_tests()
    return {
        {
            "tests-nativepython-replicator.infinigen_sdg_default",
            "standalone_examples/replicator/infinigen/infinigen_sdg.py",
            "--close-on-completion",
        },
        {
            "tests-nativepython-replicator.infinigen_sdg_config",
            "standalone_examples/replicator/infinigen/infinigen_sdg.py",
            "--close-on-completion --config standalone_examples/replicator/infinigen/config/infinigen_multi_writers_pt.yaml",
        },
        {
            "tests-nativepython-replicator.scene_based_sdg",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_basic_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            "--config standalone_examples/replicator/scene_based_sdg/config/config_basic_writer.yaml",
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_default_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            "--config standalone_examples/replicator/scene_based_sdg/config/config_default_writer.json",
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_kitti_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            "--config standalone_examples/replicator/scene_based_sdg/config/config_kitti_writer.yaml",
        },
        {
            "tests-nativepython-replicator.scene_based_sdg_coco_writer",
            "standalone_examples/replicator/scene_based_sdg/scene_based_sdg.py",
            "--config standalone_examples/replicator/scene_based_sdg/config/config_coco_writer.yaml",
        },
        {
            "tests-nativepython-replicator.object_based_sdg",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
        },
        {
            "tests-nativepython-replicator.object_based_sdg_config",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
            "--config standalone_examples/replicator/object_based_sdg/config/object_based_sdg_config.yaml",
        },
        {
            "tests-nativepython-replicator.object_based_sdg_config_dope",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
            "--config standalone_examples/replicator/object_based_sdg/config/object_based_sdg_dope_config.yaml",
        },
        {
            "tests-nativepython-replicator.object_based_sdg_config_centerpose",
            "standalone_examples/replicator/object_based_sdg/object_based_sdg.py",
            "--config standalone_examples/replicator/object_based_sdg/config/object_based_sdg_centerpose_config.yaml",
        },
        {
            "tests-nativepython-replicator.writer_augmentation_numpy",
            "standalone_examples/replicator/augmentation/writer_augmentation.py",
            "--num_frames 1",
        },
        {
            "tests-nativepython-replicator.writer_augmentation_warp",
            "standalone_examples/replicator/augmentation/writer_augmentation.py",
            "--num_frames 1 --use_warp",
        },
        {
            "tests-nativepython-replicator.annotator_augmentation_numpy",
            "standalone_examples/replicator/augmentation/annotator_augmentation.py",
            "--num_frames 1",
        },
        {
            "tests-nativepython-replicator.annotator_augmentation_warp",
            "standalone_examples/replicator/augmentation/annotator_augmentation.py",
            "--num_frames 1 --use_warp",
        },
        {
            "tests-nativepython-replicator.amr_navigation",
            "standalone_examples/replicator/amr_navigation.py",
            "--num_frames 3 --env_interval 1",
        },
        {
            "tests-nativepython-replicator.amr_navigation_use_temp_rp",
            "standalone_examples/replicator/amr_navigation.py",
            "--num_frames 3 --env_interval 1 --use_temp_rp",
        },
        {
            "tests-nativepython-replicator.cosmos_writer_warehouse",
            "standalone_examples/replicator/cosmos_writer_warehouse.py",
        },
        -- From Misc Replicator
        {
            "tests-nativepython-isaacsim.replicator.behavior.behaviors",
            "/standalone_examples/api/isaacsim.replicator.behavior/behaviors.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.cosmos_writer_simple",
            "standalone_examples/api/isaacsim.replicator.examples/cosmos_writer_simple.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.custom_event_and_write",
            "/standalone_examples/api/isaacsim.replicator.examples/custom_event_and_write.py",
        },
        {
            "tests-nativepython-testing-isaacsim.replicator.examples.ar_capture_pipeline",
            "/standalone_examples/testing/isaacsim.replicator.examples/ar_capture_pipeline.py",
        },
        {
            "tests-nativepython-testing-isaacsim.replicator.examples.ar_capture_pipeline_gpu",
            "/standalone_examples/testing/isaacsim.replicator.examples/ar_capture_pipeline.py",
            "--gpu_dynamics",
        },
        {
            "tests-nativepython-testing-isaacsim.replicator.examples.motion_blur_short",
            "/standalone_examples/api/isaacsim.replicator.examples/motion_blur.py",
            "--delta_times None 0.00416666666 --samples_per_pixel 32 --motion_blur_subsamples 4",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.subscribers_and_events",
            "/standalone_examples/api/isaacsim.replicator.examples/subscribers_and_events.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.custom_fps_writer_annotator",
            "/standalone_examples/api/isaacsim.replicator.examples/custom_fps_writer_annotator.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_01",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_01.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_02",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_02.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_03",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_03.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.sdg_getting_started_04",
            "standalone_examples/api/isaacsim.replicator.examples/sdg_getting_started_04.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.simready_assets_sdg",
            "standalone_examples/api/isaacsim.replicator.examples/simready_assets_sdg.py",
            "--num_scenarios 2",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.multi_camera",
            "standalone_examples/api/isaacsim.replicator.examples/multi_camera.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.examples.simulation_get_data",
            "standalone_examples/api/isaacsim.replicator.examples/simulation_get_data.py",
        },
        {
            "tests-nativepython-isaacsim.replicator.grasping.grasping_workflow_sdg",
            "standalone_examples/api/isaacsim.replicator.grasping/grasping_workflow_sdg.py",
        },
        {
            "tests-nativepython-testing-omni.replicator.agent.test_scripting",
            "standalone_examples/testing/omni.replicator.agent/test_scripting.py",
        },
    }
end

local function get_ros_tests()
    return {
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.enable_extension",
            "standalone_examples/testing/isaacsim.ros2.bridge/enable_extension.py",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_carter_camera_multi_robot_nav",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_carter_camera_multi_robot_nav.py",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_people_sim",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_people_sim.py",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_camera_tf_delay",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_camera_tf_delay.py",
            "--test-steps=50",
        },
        {
            "tests-nativepython-testing-isaacsim.ros2.bridge.test_publish_camera_data",
            "standalone_examples/testing/isaacsim.ros2.bridge/test_publish_camera_data.py",
            "--test-steps=5",
        },
    }
end

local function get_testing_misc_tests()
    return {
        -- Core API Testing Samples
        {
            "tests-nativepython-testing-isaacsim.core.api.test_hello_world",
            "standalone_examples/testing/isaacsim.core.api/test_hello_world.py",
            "--test",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_articulation",
            "standalone_examples/testing/isaacsim.core.api/test_articulation.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_time_stepping",
            "standalone_examples/testing/isaacsim.core.api/test_time_stepping.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_save_stage",
            "standalone_examples/testing/isaacsim.core.api/test_save_stage.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_delete_in_contact",
            "standalone_examples/testing/isaacsim.core.api/test_delete_in_contact.py",
        },
        {
            "tests-nativepython-testing-isaacsim.core.api.test_xform_prim_view",
            "standalone_examples/testing/isaacsim.core.api/test_xform_prim_view.py",
        },
        -- Python Shell Tests
        {
            "tests-nativepython-testing-python_sh.import_torch",
            "standalone_examples/testing/python_sh/import_torch.py",
        },
        {
            "tests-nativepython-testing-python_sh.import_scipy",
            "standalone_examples/testing/python_sh/import_scipy.py",
        },
        {
            "tests-nativepython-testing-python_sh.path_length",
            "standalone_examples/testing/python_sh/path_length.py",
        },
        {
            "tests-nativepython-testing-python_sh.import_sys",
            "standalone_examples/testing/python_sh/import_sys.py",
        },
        -- Tutorials
        {
            "tests-nativepython-testing-tutorials-getting_started",
            "standalone_examples/tutorials/getting_started.py",
        },
        {
            "tests-nativepython-testing-tutorials-getting_started_robot",
            "standalone_examples/tutorials/getting_started_robot.py",
        },
    }
end

local function get_benchmark_tests()
    return {
        {
            "tests-standalone_benchmarks-benchmark_camera",
            "standalone_examples/benchmarks/benchmark_camera.py",
            "--num-frames 10 --num-cameras 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_nova_carter_ros2",
            "standalone_examples/benchmarks/benchmark_robots_nova_carter_ros2.py",
            "--num-frames 10 --num-robots 2 --enable-3d-lidar 1 --enable-2d-lidar 2 --enable-hawks 1 --non-headless",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_nova_carter",
            "standalone_examples/benchmarks/benchmark_robots_nova_carter.py",
            "--num-frames 10 --num-robots 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_rtx_lidar_rotary",
            "standalone_examples/benchmarks/benchmark_rtx_lidar.py",
            "--num-frames 10 --num-sensors 8 --lidar-type Rotary",
        },
        {
            "tests-standalone_benchmarks-benchmark_rtx_lidar_solid_state",
            "standalone_examples/benchmarks/benchmark_rtx_lidar.py",
            "--num-frames 10 --num-sensors 8 --lidar-type Solid_State",
        },
        {
            "tests-standalone_benchmarks-benchmark_sdg_simple",
            "standalone_examples/benchmarks/benchmark_sdg.py",
            "--num-frames 10 --num-cameras 2 --resolution 1280 720 --asset-count 10 --annotators rgb distance_to_camera --disable-viewport-rendering --delete-data-when-done --headless --print-results",
        },
        {
            "tests-standalone_benchmarks-benchmark_sdg_advanced",
            "standalone_examples/benchmarks/benchmark_sdg.py",
            "--num-frames 10 --num-cameras 2 --resolution 1280 720 --asset-count 10 --annotators all --disable-viewport-rendering --delete-data-when-done --headless --print-results",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_ur10",
            "standalone_examples/benchmarks/benchmark_robots_ur10.py",
            "--num-frames 10 --num-robots 10",
        },
        {
            "tests-standalone_benchmarks-benchmark_physx_lidar",
            "standalone_examples/benchmarks/benchmark_physx_lidar.py",
            "--num-frames 10 --num-sensors 4",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_o3dyn",
            "standalone_examples/benchmarks/benchmark_robots_o3dyn.py",
            "--num-frames 10 --num-robots 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_scene_loading",
            "standalone_examples/benchmarks/benchmark_scene_loading.py",
            "--num-frames 10 --env-url /Isaac/Environments/Simple_Warehouse/full_warehouse.usd",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_evobot",
            "standalone_examples/benchmarks/benchmark_robots_evobot.py",
            "--num-frames 10 --num-robots 1 1 1",
        },
        {
            "tests-standalone_benchmarks-benchmark_single_view_depth_sensor",
            "standalone_examples/benchmarks/benchmark_single_view_depth_sensor.py",
            "--num-frames 10 --num-cameras 2",
        },
        {
            "tests-standalone_benchmarks-benchmark_robots_humanoid",
            "standalone_examples/benchmarks/benchmark_robots_humanoid.py",
            "--num-frames 10 --num-robots 2",
        },
    }
end

function create_tests()
    -- Startup test group
    define_test_group("startup_tests", get_startup_tests())

    -- Python samples
    group("python_samples")

    -- smoke tests for python.sh itself
    python_script_test("tests-nativepython-pip_list", "-m pip list --")
    python_script_test(
        "tests-nativepython-pycocotools",
        "-m pip install --force pycocotools --no-cache-dir --no-dependencies --"
    ) -- this test makes sure that pip packages that need Python.h can be installed.

    -- Omni Kit App
    python_sample_test(
        "tests-nativepython-omni.kit.app.app_framework",
        "standalone_examples/api/omni.kit.app/app_framework.py"
    )

    register_python_sample_tests(get_simulation_app_tests())

    if os.target() == "linux" then
        python_sample_test(
            "tests-nativepython-testing-isaacsim.simulation_app.test_ovd",
            "standalone_examples/testing/isaacsim.simulation_app/test_ovd.py",
            '--ovd="/tmp/"'
        )
    end

    register_python_sample_tests(get_core_tests())
    register_python_sample_tests(get_sensor_tests())
    register_python_sample_tests(get_robot_tests())
    register_python_sample_tests(get_asset_tests())
    register_python_sample_tests(get_replicator_tests())
    register_python_sample_tests(get_ros_tests())
    register_python_sample_tests(get_testing_misc_tests())

    -- Benchmarks
    group("benchmarks")
    register_python_sample_tests(get_benchmark_tests())

end
