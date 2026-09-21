from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="wall_follower",
                executable="wall_follower",
                name="wall_follower",
            ),
            Node(
                package="collision_avoidance",
                executable="collision_avoidance",
                name="collision_avoidance",
            ),
            Node(
                package="fsm_node",
                executable="fsm_node",
                name="fsm_supervisor",
            ),
        ]
    )
