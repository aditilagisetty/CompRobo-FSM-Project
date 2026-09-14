# RoboBehaviors and Finite State Machines Project

Author: Aditi Lagisetty

For Olin ENGR3590 Computational Introduction to Robotics

## Project Overview

TODO: overall description of the project, motivation (link to the
assignment prompt), key design choices, and a summary of key takeaways.

## Individual Behaviors

In this section you will provide the details of each individual behavior you
created / is part of this repository. At a minimum, you should have an
emergency stop, driving in a shape, and wall-following. Any additional
behaviors you created as part of your finite-state-machine should also be
described here.

### Behavior 1: Drive in a Square

- **What it does:** [drive_square.py](ros_behaviors_fsm/ros_behaviors_fsm/drive_square.py)
  drives the Neato through a 1m x 1m square path using time-based control
  (drive forward for a fixed duration, turn 90 degrees, repeat 4 times). It
  also subscribes to an `estop` topic and to `scan` so it can stop early if
  something is within `stop_distance` in front of the robot.
- **Implementation details:** TODO -- expand on this once you've adapted the
  behavior further. Currently: subscribes to `estop` (`std_msgs/Bool`) and
  `scan` (`sensor_msgs/LaserScan`), publishes to `cmd_vel`
  (`geometry_msgs/Twist`). The driving logic runs on its own thread, separate
  from the callbacks that watch for the e-stop condition (see the threading
  discussion below).
- **Design decisions:**
  - **Single- vs. multi-threaded:** We compared two reference
    implementations before adapting one:
    [drive_square_single_threaded.py](ros_behaviors_fsm/ros_behaviors_fsm/drive_square_single_threaded.py)
    (timer-callback based, single-threaded) and
    [drive_square.py](ros_behaviors_fsm/ros_behaviors_fsm/drive_square.py)
    (dedicated Thread running the drive sequence, multi-threaded). We chose
    the multi-threaded approach because the drive sequence needs to react to
    an asynchronous e-stop condition (bump/obstacle) coming in on a separate
    callback -- a pure timer-based loop can't be interrupted mid-segment
    without extra bookkeeping, whereas a `threading.Event` shared between the
    driving thread and the ROS callbacks handles that naturally.
  - Annotating the two samples surfaced two bugs worth flagging: the
    single-threaded sample never stops after the fourth side (nothing checks
    `turns_executed` to end the loop), and the multi-threaded sample's e-stop
    can never be cleared once triggered (`handle_estop` only ever calls
    `e_stop.set()`, never `.clear()`). TODO: fix these before relying on
    either as the final behavior.
- **Demo:** TODO -- add a gif/video and link to `bags/drive_square_demo`.

### Behavior 2: Collision Avoidance

TODO. Starting point: [collision_avoidance.py](ros_behaviors_fsm/ros_behaviors_fsm/collision_avoidance.py)
(skeleton combining the day 3 bump-triggered
[emergency_stop.py](ros_behaviors_fsm/ros_behaviors_fsm/emergency_stop.py) and
proximity-triggered
[distance_emergency_stop.py](ros_behaviors_fsm/ros_behaviors_fsm/distance_emergency_stop.py)
approaches).

### Behavior 3: Wall Following

TODO. Starting point: [wall_follower.py](ros_behaviors_fsm/ros_behaviors_fsm/wall_follower.py)
(skeleton only -- proportional control logic not yet implemented).

## Finite State Machine

### Overall Design

TODO -- flesh out once behaviors 2 and 3 are implemented. Draft design is
sketched in [finite_state_controller.py](ros_behaviors_fsm/ros_behaviors_fsm/finite_state_controller.py):

- **States:** `DRIVE_SQUARE` (default), `COLLISION_AVOIDANCE` (entered on
  bump or close-range obstacle), `WALL_FOLLOWING` (entered when a wall is
  detected nearby).
- **Transitions:** TODO -- define the actual sensor thresholds/logic for
  each transition listed in the skeleton file's docstring.

### Implementation Details

TODO -- pointers to relevant code once the FSM is implemented.

### Demonstration

TODO -- add a gif/video and link to `bags/finite_state_controller_demo`.

## Learning Objectives and Final Takeaways

TODO.

## How To Run

1. Clone this repo into the `src/` directory of a ROS2 workspace.
2. From the workspace root: `colcon build --packages-select ros_behaviors_fsm`
3. `source install/setup.bash`
4. Run an individual behavior, e.g. `ros2 run ros_behaviors_fsm drive_square`
5. Play back a recorded run: `ros2 bag play bags/<bag-name> --clock`
