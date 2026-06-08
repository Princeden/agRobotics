# Warthog Navigate-and-Manipulate Package — Plan

A ROS 2 package for the Clearpath W200 (Warthog) + UR5e that drives the base to a
given coordinate (Nav2) and then commands the arm (MoveIt). This document covers
both **how to write the module** and **how to test it in Gazebo**.

Target stack: ROS 2 Humble, `clearpath_gz` simulation, namespace `w200_0000`.

---

## 1. Architecture

The package does **not** implement navigation or motion planning. Those already run
as servers:
- Nav2 provides the `navigate_to_pose` action.
- MoveIt `move_group` provides the `move_action` action (planning group `arm_0`).

Our package is a **coordinator**: one node that is a *client* of both and exposes a
high-level interface ("go to (x, y, θ), then move the arm to pose P").

```
            ┌─────────────────────────────────────────┐
   you ───► │  task_coordinator  (our node)            │
 (goal)     │   • action server: GoToAndManipulate     │
            └───┬───────────────────────────────┬──────┘
                │ NavigateToPose (action)        │ MoveGroup (pymoveit2)
                ▼                                ▼
          Nav2 (bt_navigator,            MoveIt move_group
          planner, controller)          (plans → arm_0_joint_trajectory_controller)
                │                                │
                ▼                                ▼
          /w200_0000/cmd_vel            arm_0_joint_trajectory_controller
          (diff_drive_controller)       (the 6 UR5e joints)
```

The coordinator's value is **sequencing and failure handling** in one place: tuck the
arm before driving, gate the arm motion on navigation success, abort cleanly on failure.

### What the stack exposes

| Subsystem | What's running | How we command it |
|---|---|---|
| **Base** | `diff_drive_controller`, EKF localization (`localization.yaml`) | Publish `geometry_msgs/Twist` on `/w200_0000/cmd_vel` (lowest-priority twist_mux input). Odometry via TF `odom → base_link`. |
| **Arm** | MoveIt `move_group` + `arm_0_joint_trajectory_controller`, planning group `arm_0` (6 joints) | MoveIt via `pymoveit2`, or direct `FollowJointTrajectory` action |
| **Nav2** | Not installed by default | `apt install ros-humble-navigation2 ros-humble-nav2-bringup` |

Arm joints (group `arm_0`):
`arm_0_shoulder_pan_joint, arm_0_shoulder_lift_joint, arm_0_elbow_joint,
arm_0_wrist_1_joint, arm_0_wrist_2_joint, arm_0_wrist_3_joint`
Arm base link: `arm_0_base_link`. End effector: `arm_0_tool0`.

---

## 2. Writing the module

### 2.1 Package layout

Python (`ament_python`) is the path of least resistance:

```
warthog_task/
├── package.xml
├── setup.py
├── warthog_task/
│   ├── __init__.py
│   └── task_coordinator.py        # the node
└── launch/
    └── task.launch.py
```

Custom action interfaces can't live cleanly in a pure `ament_python` package. Two options:
- **(a)** Put `GoToAndManipulate.action` in a small separate `warthog_task_msgs`
  (`ament_cmake`) package — the standard approach.
- **(b)** For a first version, skip the custom action: have the coordinator accept a
  `geometry_msgs/PoseStamped` target on a topic/service and hardcode the arm target.
  Start with (b).

### 2.2 Dependencies (`package.xml`)

```xml
<exec_depend>rclpy</exec_depend>
<exec_depend>geometry_msgs</exec_depend>
<exec_depend>nav2_msgs</exec_depend>          <!-- NavigateToPose -->
<exec_depend>tf_transformations</exec_depend> <!-- yaw → quaternion -->
<exec_depend>pymoveit2</exec_depend>          <!-- or moveit_py / C++ move_group_interface -->
```

### 2.3 Base half — Nav2 client

Nav2's entry point is the `navigate_to_pose` action (`nav2_msgs/action/NavigateToPose`).
The goal is a `PoseStamped` **in the `map` frame**. Under the `w200_0000` namespace the
action name resolves to `/w200_0000/navigate_to_pose`.

```python
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from tf_transformations import quaternion_from_euler

class TaskCoordinator(Node):
    def __init__(self):
        super().__init__('task_coordinator')
        self._nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    async def drive_to(self, x, y, yaw):
        self._nav.wait_for_server()
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        qx, qy, qz, qw = quaternion_from_euler(0, 0, yaw)
        goal.pose.pose.orientation.x = qx
        goal.pose.pose.orientation.y = qy
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        handle = await self._nav.send_goal_async(goal)
        if not handle.accepted:
            raise RuntimeError('Nav2 rejected the goal')
        result = await handle.get_result_async()
        return result.status   # 4 == SUCCEEDED
```

**Nav2 setup required (the bulk of the effort):**
1. Install: `sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup`.
2. **Map + localization.** Nav2 needs `map → odom`. The stack already gives
   `odom → base_link` (EKF in `localization.yaml`). Add AMCL (pre-built map) or
   `slam_toolbox` (build live) to provide `map → odom`.
3. **Nav2 params file** with: `w200_0000` namespace, `global_frame: map`,
   `robot_base_frame: base_link`, `odom_topic` → EKF output, `cmd_vel` remapped to
   the diff-drive input, Warthog footprint (~1.35 × 0.7 m), `use_sim_time: true`.
4. Launch `nav2_bringup` under the namespace.
5. **Use `clearpath_nav2_demos` as a params/launch template** rather than writing from scratch.

### 2.4 Arm half — MoveIt client (`pymoveit2`)

`move_group` runs with planning group `arm_0` and `arm_0_joint_trajectory_controller`
registered as its execution controller.

```python
from pymoveit2 import MoveIt2
from rclpy.callback_groups import ReentrantCallbackGroup

self._moveit2 = MoveIt2(
    node=self,
    joint_names=[f'arm_0_{j}' for j in (
        'shoulder_pan_joint','shoulder_lift_joint','elbow_joint',
        'wrist_1_joint','wrist_2_joint','wrist_3_joint')],
    base_link_name='arm_0_base_link',
    end_effector_name='arm_0_tool0',
    group_name='arm_0',
    callback_group=ReentrantCallbackGroup(),
)

# joint-space goal (safe, deterministic):
self._moveit2.move_to_configuration([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
self._moveit2.wait_until_executed()

# OR cartesian pose goal, in the ARM base frame (not map!):
self._moveit2.move_to_pose(position=[0.4, 0.0, 0.3],
                           quat_xyzw=[0.0, 1.0, 0.0, 0.0],
                           frame_id='arm_0_base_link')
self._moveit2.wait_until_executed()
```

Alternatives (same `move_group` backend): `moveit_py` (official) or C++
`MoveGroupInterface` (most robust). Start with `pymoveit2` to keep it in one node.

**Prerequisite:** MoveIt must be launched. Enable it in `robot.yaml`:
```yaml
manipulators:
  moveit:
    enable: true
  arms:
    - model: universal_robots
      ...
```
Then regenerate the description. Arm targets are most reliable in `arm_0_base_link`.

### 2.5 Sequencing (the coordinator's actual logic)

```python
async def execute(self, x, y, yaw, arm_joint_goal):
    # 1. tuck the arm before driving (collision/tip safety)
    self._moveit2.move_to_configuration(TRAVEL_POSE); self._moveit2.wait_until_executed()
    # 2. navigate
    status = await self.drive_to(x, y, yaw)
    if status != 4:  # STATUS_SUCCEEDED
        self.get_logger().error('navigation failed; aborting arm motion'); return
    # 3. manipulate
    self._moveit2.move_to_configuration(arm_joint_goal); self._moveit2.wait_until_executed()
```

### 2.6 Runtime gotchas

- **`MultiThreadedExecutor` + `ReentrantCallbackGroup`.** Awaiting one action's result
  while another node's callbacks (TF, MoveIt feedback) must keep spinning — a
  single-threaded executor deadlocks.
- **`use_sim_time: true`** on every node in Gazebo, or action timeouts / TF lookups misbehave.
- **Namespacing:** launch the node inside `w200_0000` (or remap) so `navigate_to_pose`,
  `move_action`, and `cmd_vel` resolve automatically. MoveIt internal topics sometimes
  need explicit remapping under a namespace — the #1 "move_group not found" cause.
- **Frames discipline:** base goals → `map`; arm goals → `arm_0_base_link`. Keep separate.

---

## 3. Testing in Gazebo

Golden rule: **bring each layer up, prove it by hand, then add the next.** By the time
the coordinator runs, navigation and arm motion are each independently proven, so any
failure is unambiguously in the coordinator's sequencing logic.

### Layer 0 — Launch the sim, confirm robot + arm spawn

```bash
ros2 launch clearpath_gz simulation.launch.py \
    setup_path:=/home/phom/agrobotics_ws/robot_yaml \
    world:=warehouse \
    rviz:=true
```
`warehouse` = obstacles for Nav2; `empty` = pure arm testing. All under `/w200_0000`,
`use_sim_time:=true` already set.

```bash
ros2 control list_controllers -c /w200_0000/controller_manager
# expect: platform_velocity_controller (active),
#         arm_0_joint_trajectory_controller (active),
#         joint_state_broadcaster (active)
ros2 topic echo /w200_0000/platform/odom --once   # base odometry flowing
```

### Layer 1 — Drive the base by hand (no Nav2)

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -r /cmd_vel:=/w200_0000/cmd_vel
```
Robot should move. Proves `cmd_vel` → motion; any later Nav2 issue is then a config issue.

### Layer 2 — Arm via MoveIt, from RViz

```bash
ros2 node list | grep move_group   # must exist; if not, enable moveit in robot.yaml + regenerate
```
In RViz **MotionPlanning** panel: drag the marker → **Plan & Execute** → arm moves in Gazebo.
Then headless smoke test:
```bash
ros2 action list | grep -E "move_action|follow_joint_trajectory"
ros2 action send_goal /w200_0000/arm_0_joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory "{ ... }"
```

### Layer 3 — Nav2 + a map

Nav2 needs `map → odom`, not provided by the sim alone:
- **SLAM (easiest start):** run `slam_toolbox` in `/w200_0000`, drive with teleop to build a map live.
- **AMCL:** save that map, localize against it later.

```bash
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=true namespace:=/w200_0000 \
    params_file:=<your_nav2_params.yaml>
```
Params: `global_frame: map`, `robot_base_frame: base_link`, Warthog footprint
(~1.35 × 0.7 m), `cmd_vel` → `/w200_0000/cmd_vel`. Template off `clearpath_nav2_demos`.

Test **without the node** — RViz "Nav2 Goal" tool, or:
```bash
ros2 action send_goal /w200_0000/navigate_to_pose nav2_msgs/action/NavigateToPose \
    "{pose: {header: {frame_id: map}, pose: {position: {x: 3.0, y: 1.0}, orientation: {w: 1.0}}}}"
```

### Layer 4 — The coordinator node

```bash
ros2 run warthog_task task_coordinator --ros-args -p use_sim_time:=true -r __ns:=/w200_0000
```
Watch Gazebo (motion), RViz (paths + arm previews), and:
```bash
ros2 action list                       # GoToAndManipulate should appear
ros2 topic echo /w200_0000/cmd_vel     # confirm nav commands the base
```

### Gazebo testing tips

- **Tune in `world:=empty` first**, then switch to `warehouse`.
- **`use_sim_time:=true` on every node** (coordinator, teleop, Nav2, SLAM). #1 gotcha —
  a single wall-clock node causes TF extrapolation errors and silent action hangs.
- **Reset cheaply:** Gazebo reset / relaunch; keep RViz open across runs.
- **Tuck the arm before nav tests** — an extended UR5e changes the footprint, can clip shelves.
- **Arm plans but doesn't move in Gazebo** → almost always controller/`use_sim_time`
  mismatch or the gz↔ros2_control bridge, not MoveIt. Check `ros2 control list_controllers`
  and joint_states flow.

---

## 4. Suggested build order

1. **MoveIt standalone** — confirm `move_group` up, plan from RViz, reproduce with a
   short `pymoveit2` script. (Quickest win, no nav dependency.)
2. **Nav2 standalone** — bring up SLAM/AMCL + nav2, send a goal from RViz. Validates
   map/TF/params before any code.
3. **Coordinator** — chain the two action clients. Both halves already proven, so only
   the sequencing logic is new.
