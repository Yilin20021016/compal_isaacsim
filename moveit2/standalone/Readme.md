# ROS2 MoveIt 2 & Isaac Sim Integration Guide

This guide provides a step-by-step walkthrough for setting up and running the Kinova Kortex robotic arm simulation using MoveIt 2 and NVIDIA Isaac Sim.

## Prerequisites & Setup
1. Extract Workspace Archives
    Unzip the provided project files into your main `isaacsim` directory:
    ```Bash
    # Ensure both zip files are extracted to your Isaac Sim root directory
    unzip moveit2.zip -d /path/to/isaacsim/
    unzip ros2_kortex_ws.zip -d /path/to/isaacsim/
    ```

## Execution Steps
2. Launch Isaac Sim (Standalone Simulation)
    Navigate to the `moveit2` directory and execute the standalone Python script to start Isaac Sim with random scene generation:

    ```Bash
    cd /isaacsim/moveit2
    /isaacsim/_build/linux-x86_64/release/python.sh standalone/standalone.py
    ```

3. Configure and Source the ROS2 Workspace
    Open a __new terminal__ to activate your ROS2 Kinova Kortex workspace:

    ```Bash
    cd /path/to/isaacsim/ros2_kortex_ws/Kortex
    source install/setup.bash
    ```
    💡 __Troubleshooting__: If you encounter environment or sourcing issues during this step, please ensure MoveIt 2 is correctly installed on your system, then clean and rebuild the workspace:

    ```Bash
    colcon build --symlink-install
    source install/setup.bash
    ```

4. Launch Isaac Bridge with MoveIt 2
    To synchronize the simulation control, launch the `isaac_bridge` node:

    ``` Bash
    ros2 launch isaac_bridge isaac_bridge.launch.py
    ```

    - This command automatically launches RViz 2 and establishes a bidirectional bridge mapping MoveIt 2 joint states (`/joint_states`) directly to Isaac Sim OmniGraph joint commands (`/isaac_joint_command`).

### Alternative: Run the Bridge Alone
If you only need to run the core bridge node without launching full visualization or auxiliary configs, use:

```Bash
ros2 run isaac_bridge moveit_bridge
```