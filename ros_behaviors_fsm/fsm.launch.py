from launch import LaunchDescription
from launch_ros.actions import Node

PACKAGE = "ros_behaviors_fsm"


def generate_launch_description():
    """Generate a launch description for the FSM node and its dependencies."""
    return LaunchDescription(
        [
            Node(
                package=PACKAGE,
                executable="fsm_node",
                name="fsm_node",
                output="screen",
                # keyboard_listener needs a real terminal (raw stdin) --
                # run this one directly with `ros2 run`, not inside a
                # launch file, if it stops reading key presses correctly.
                emulate_tty=True,
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
            ),
            Node(
                package=PACKAGE,
                executable="drive_square",
                name="drive_square",
                output="screen",
            ),
        ]
    )
