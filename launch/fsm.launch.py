"""
Bring up the gateway FSM and every behavior node that feeds it: fsm_node,
wall_follower, collision_avoidance, path_following, teleop_scan, and
drive_square. Include this against an already-running world, or use
bringup.launch.py to also start Gazebo first.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PACKAGE = "ros_behaviors_fsm"


def generate_launch_description():
    """
    Generate a launch description for the FSM node and its dependencies.

    fsm_node and teleop_scan both read raw key presses straight from stdin,
    and ros2 launch doesn't give more than one node under the same launch
    file a real, usable interactive terminal. Pass
    with_keyboard_nodes:=false to skip both of them and launch the rest
    together then run

        ros2 run ros_behaviors_fsm teleop_scan
        ros2 run ros_behaviors_fsm fsm_node

    by hand, each in its own terminal, for working keyboard control.
    """
    with_keyboard_nodes_arg = DeclareLaunchArgument(
        "with_keyboard_nodes",
        default_value="true",
        description="Also launch fsm_node and teleop_scan (they need a "
        "real terminal for keyboard input -- set to false to skip them "
        "and run them by hand instead).",
    )
    with_keyboard_nodes = IfCondition(LaunchConfiguration("with_keyboard_nodes"))

    return LaunchDescription(
        [
            with_keyboard_nodes_arg,
            Node(
                package=PACKAGE,
                executable="fsm_node",
                name="fsm_node",
                output="screen",
                # keyboard_listener needs a real terminal (raw stdin) --
                # run this one directly with `ros2 run`, not inside a
                # launch file, if it stops reading key presses correctly.
                emulate_tty=True,
                condition=with_keyboard_nodes,
            ),
            Node(
                package=PACKAGE,
                executable="path_following",
                name="path_following",
                output="screen",
            ),
            Node(
                package=PACKAGE,
                executable="wall_follower",
                name="wall_follower",
                output="screen",
            ),
            Node(
                package=PACKAGE,
                executable="collision_avoidance",
                name="collision_avoidance",
                output="screen",
            ),
            Node(
                package=PACKAGE,
                executable="teleop_scan",
                name="teleop_scan",
                output="screen",
                emulate_tty=True,
                condition=with_keyboard_nodes,
            ),
            Node(
                package=PACKAGE,
                executable="drive_square",
                name="drive_square",
                output="screen",
            ),
        ]
    )
