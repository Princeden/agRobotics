# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A single ROS 2 (Humble) `ament_cmake` package, `ur_simulation_gazebo`, providing example launch files and configuration for **Gazebo Classic** simulation of Universal Robots arms. It ships no compiled code — only launch files, xacro/URDF descriptions, YAML configs, RViz configs, and worlds that are installed into `share/`. The actual robot model, controllers, and MoveIt config come from upstream packages (`ur_description`, `ur_controllers`, `ur_moveit_config`), which must be present in the workspace or installed.

Gazebo Classic is EOL from ROS 2 Jazzy onward, so this package is Humble-only.

## Build, test, run

This package lives inside a colcon workspace (the repo is cloned under `<ws>/src/`). All commands run from the workspace root, not this directory.

```bash
# Build (from workspace root, after sourcing /opt/ros/humble/setup.bash)
colcon build --symlink-install --packages-select ur_simulation_gazebo
source install/setup.bash

# Install missing upstream deps (ur_description, ur_moveit_config, realsense2_description, ...)
rosdep install --ignore-src --from-paths src -y

# Run the full launch test (spins up Gazebo headless; 180s timeout)
colcon test --packages-select ur_simulation_gazebo
colcon test-result --verbose         # view failures

# Run the launch test directly (faster iteration than colcon test)
launch_test src/Universal_Robots_ROS2_Gazebo_Simulation/ur_simulation_gazebo/test/test_gazebo.py
```

Launch the simulation (requires the workspace to be built and sourced):

```bash
ros2 launch ur_simulation_gazebo ur_sim_control.launch.py   # Gazebo + controllers + RViz + MoveIt
ros2 launch ur_simulation_gazebo ur_sim_moveit.launch.py    # adds MoveIt motion planning on top
```

Common launch args: `ur_type:=ur5e` (default), `description_file:=...`, `world:=<abs path>`, `launch_rviz:=false`, `gazebo_gui:=false`, `initial_positions_file:=<abs path>`.

## Lint

Linting is driven by `pre-commit` (black, flake8, codespell, the `ament_*` hooks, clang-format, doc8, etc.):

```bash
pre-commit run -a
```

CI mirrors this (`pre-commit`, `ros-lint`) plus binary/source builds against Humble. See `ci_status.md` for the full workflow matrix.

## Architecture / launch flow

The two launch files form a layered chain — read them together to understand a run:

- **`ur_sim_control.launch.py`** is the core. In `launch_setup()` it:
  1. Builds `robot_description` by invoking `xacro` on `description_file` (from `description_package`), passing `sim_gazebo:=true`, the resolved controllers file, safety args, and the initial-positions file.
  2. Starts `robot_state_publisher`, the `gazebo_ros` `gazebo.launch.py` (with the `world` arg), and `spawn_entity.py` to inject the robot from the `/robot_description` topic.
  3. Spawns controllers via `controller_manager`'s `spawner`: always `joint_state_broadcaster`, plus `initial_joint_controller` (default `joint_trajectory_controller`) either started or `--stopped` depending on `start_joint_controller`.
  4. RViz is started on an `OnProcessExit` event handler after the joint-state broadcaster.
- **`ur_sim_moveit.launch.py`** includes `ur_sim_control.launch.py` and layers `ur_moveit_config`'s `ur_moveit.launch.py` on top for planning.

The xacro chain is the key indirection: `urdf/ur_with_camera.urdf.xacro` `xacro:include`s the stock `ur_description/urdf/ur.urdf.xacro` (so all UR args pass straight through) and bolts on a RealSense D435 from `realsense2_description`, simulated via the `libgazebo_ros_camera.so` Gazebo plugin. Argument `camera_sensor_type` selects `camera` (RGB-only, robust — the default) vs `depth` (adds depth + point cloud but needs offscreen depth rendering that can crash on some GPUs). Camera topics publish under `/camera/color/...`.

Controllers are defined in `config/ur_controllers.yaml`; per-robot kinematics/limits/geometry live under `config/<ur_type>/` (e.g. `config/ur5e/`).

## Local customizations vs. upstream

This is a customized fork of `UniversalRobots/Universal_Robots_ROS2_Gazebo_Simulation`. Working-tree changes (not yet committed) diverge from upstream in ways worth knowing before editing:

- `description_file` now defaults to **`ur_with_camera.urdf.xacro`** (wrist-mounted RealSense D435), not the plain UR arm.
- A `world` launch arg, an `rviz/view_robot_camera.rviz` config, and a sample `worlds/example.world` were added; pass a world with `world:=$(ros2 pkg prefix ur_simulation_gazebo)/share/ur_simulation_gazebo/worlds/example.world`.
- `CMakeLists.txt` installs the new `rviz urdf worlds` directories; `package.xml` adds the `realsense2_description` dependency.
- `ur_sim_control.launch.py` also includes MoveIt directly (the upstream version does not). Note there are duplicated `rviz_node` definitions and a commented-out `delayed_rviz`/`moveit` block left in `launch_setup()` — clean these up if touching that file.
