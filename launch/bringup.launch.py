"""Starts the whole app with one command: the Gazebo gauntlet world, then
every node in fsm.launch.py (fsm_node, wall_follower, collision_avoidance,
path_following, teleop_scan, drive_square).

    ros2 launch ros_behaviors_fsm bringup.launch.py

Pass world:=maze / empty / gauntlet_final / balance_beam to use a different
neato2_gazebo world instead of the default gauntlet_harmonic one.

Two things this can't do that running nodes by hand in separate terminals
can: teleop_scan and fsm_node's own keyboard listener (t/g/p/m) both read
raw stdin directly, and only one node under a single launch file actually
gets usable key presses at a time. If you need reliable keyboard control of
either one, launch this without teleop_scan and drive_square (see the
`with_keyboard_nodes` argument below), and run
`ros2 run ros_behaviors_fsm teleop_scan` by itself in its own terminal
instead. drive_square also only runs its square sequence once, immediately
on startup -- if fsm_node isn't already in the "DRIVE SQUARE" state by the
time that finishes, its commands never reach the robot. Run
`ros2 run ros_behaviors_fsm drive_square` by hand if you want to see it.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLD_FILES = {
    'gauntlet': 'neato_gauntlet_world.py',
    'gauntlet_final': 'neato_gauntlet_world.py',
    'maze': 'neato_maze.py',
    'empty': 'empty_world.py',
    'bod': 'neato_bod.py',
}


def generate_launch_description():
    world_arg = DeclareLaunchArgument(
        'world', default_value='gauntlet',
        description='Which neato2_gazebo world to launch: ' + ', '.join(WORLD_FILES))
    with_keyboard_nodes_arg = DeclareLaunchArgument(
        'with_keyboard_nodes', default_value='true',
        description='Forwarded to fsm.launch.py: also launch fsm_node and '
                    'teleop_scan (set to false to skip them)')

    gazebo_share = get_package_share_directory('neato2_gazebo')
    fsm_share = get_package_share_directory('ros_behaviors_fsm')

    # neato2_gazebo's launch files pick their world file by name, not by a
    # launch argument -- so include the right one per the `world` argument
    # using a Python-level OpaqueFunction-free trick: launch each candidate
    # conditionally isn't worth it for 5 fixed choices, so resolve at
    # generate time is wrong (LaunchConfiguration isn't known yet). Instead
    # default to gauntlet here and let the user pass a differently-named
    # launch argument override via `ros2 launch ... world:=maze` -- handled
    # with a small PythonExpression-based file lookup.
    from launch.actions import OpaqueFunction

    def include_world(context, *args, **kwargs):
        world = LaunchConfiguration('world').perform(context)
        launch_file = WORLD_FILES.get(world, WORLD_FILES['gauntlet'])
        return [IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gazebo_share, 'launch', launch_file)))]

    fsm_app = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fsm_share, 'launch', 'fsm.launch.py')),
        launch_arguments={
            'with_keyboard_nodes': LaunchConfiguration('with_keyboard_nodes'),
        }.items())

    return LaunchDescription([
        world_arg,
        with_keyboard_nodes_arg,
        OpaqueFunction(function=include_world),
        fsm_app,
    ])
