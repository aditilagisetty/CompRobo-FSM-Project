# CompRobo FSM Project

A ROS 2 Jazzy / Gazebo Harmonic project for Olin's Computational Robotics
class. A Neato vacuum robot is driven by a set of independent behavior
nodes (drive-a-square, wall following, obstacle avoidance, keyboard
teleop + mapping, and path following), all gated through a single
finite-state-machine "gateway" node that decides which behavior's
velocity commands actually reach the robot at any given moment.

## Team

- Duc Tuan Nguyen
- Aditi Lagisetty
- Akil Pugalenthi

## Demo video

[Watch on YouTube](https://www.youtube.com/watch?v=WOOGdodrU7g)

A local copy of the same walkthrough is also in
[`videos/screencast/full_demo.mp4`](videos/screencast/full_demo.mp4),
and six annotated rosbags of each behavior running standalone live in
[`bags/`](bags/).

## How it works

`fsm_node` is the gateway: every behavior node publishes its velocity
commands to its own topic (`cmd_vel_wall_follower`, `cmd_vel_drive_square`,
etc.) instead of `/cmd_vel` directly. `fsm_node` tracks a current mode and
forwards only the matching topic's `Twist` messages to `/cmd_vel`, so
behaviors can run side-by-side without fighting over the robot.

| Mode | Behavior node | How to enter it |
|---|---|---|
| `WALL FOLLOW` | `wall_follower` | default at startup, or press `g` |
| `OBSTACLE AVOIDANCE` | `collision_avoidance` | automatic -- `fsm_node` switches into it whenever something is too close or the robot is stuck turning in `WALL FOLLOW`, and back once it's clear |
| `TELEOP SCAN` | `teleop_scan` | press `t` |
| `DRIVE SQUARE` | `drive_square` | press `m` (only while in `TELEOP SCAN`) |
| `PATH FOLLOWING` | `path_following` | press `p` |

Mode switches can come from the keyboard (when a node has a real
terminal) or from publishing a single character to `/fsm_command`
(`ros2 topic pub -1 /fsm_command std_msgs/String "data: p"`), which
works the same way under `ros2 launch` where stdin isn't interactive.

`teleop_scan` also builds an occupancy-grid map of the room from the
lidar as you drive it around, and saves it to `~/.ros/room_map.yaml` on
command. `path_following` loads that saved map, opens an always-visible
Tk control panel (mode buttons, live status, arrow-key driving, and a
drawing window), and can either follow a route you draw by hand or plan
one with A* to a right-clicked goal.

## Prerequisites

- ROS 2 Jazzy
- Gazebo Harmonic (`gz sim`)
- The `neato2_gazebo` and `neato2_interfaces` packages in the same
  workspace as this one

## Building

```bash
cd ~/ros2_ws
colcon build --packages-select ros_behaviors_fsm
source install/setup.bash
```

## Running

The easiest way to see everything at once is the combined launch file,
which starts Gazebo and every behavior node together:

```bash
ros2 launch ros_behaviors_fsm bringup.launch.py
```

Pass a different world with `world:=maze` / `empty` / `bod` /
`gauntlet_final` (default is `gauntlet`).

**Keyboard control note:** `fsm_node` and `teleop_scan` both read raw
key presses from stdin, and `ros2 launch` only gives one node under a
launch file a real, usable terminal at a time. For working keyboard
control, launch everything else with keyboard nodes turned off, then
run those two by hand, each in its own terminal:

```bash
ros2 launch ros_behaviors_fsm bringup.launch.py with_keyboard_nodes:=false
```

```bash
# terminal 2
ros2 run ros_behaviors_fsm teleop_scan
```

```bash
# terminal 3
ros2 run ros_behaviors_fsm fsm_node
```

If you already have Gazebo running separately, use `fsm.launch.py`
instead of `bringup.launch.py` to start just the FSM and behavior nodes
against it.

### Running a single behavior standalone

Each behavior node publishes to its own `cmd_vel_<behavior>` topic by
design, so to drive the robot directly with one node (without the
gateway) remap its output to `/cmd_vel`:

```bash
ros2 run ros_behaviors_fsm wall_follower --ros-args -r cmd_vel_wall_follower:=cmd_vel
ros2 run ros_behaviors_fsm collision_avoidance --ros-args -r cmd_vel_collision_avoidance:=cmd_vel
ros2 run ros_behaviors_fsm drive_square --ros-args -r cmd_vel_drive_square:=cmd_vel
```

### Optional: drift correction

`icp_localizer` corrects odometry drift by matching lidar scans against
a saved map with ICP, and publishes the corrected `map -> odom`
transform. It isn't wired into either launch file; run it by hand
alongside the other nodes if you want it:

```bash
ros2 run ros_behaviors_fsm icp_localizer
```

## Rosbags

`bags/` has one recorded demo per behavior (`drive_square_demo`,
`collision_avoidance_demo`, `wall_follower_demo`, `teleop_scan_demo`,
`path_following_demo`, `fsm_node_demo`). Inspect any of them with:

```bash
ros2 bag info bags/<name>
ros2 bag play bags/<name>
```
