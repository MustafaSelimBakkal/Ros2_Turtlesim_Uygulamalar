from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    turtlesim_node = Node(
        package="turtlesim",
        executable="turtlesim_node"
    )

    spawn_turtle_node = Node(
        package="turtlesim_py_pkg",
        executable="spawn_turtle",
    )

    go_to_loc_node = Node(
        package="turtlesim_py_pkg",
        executable="go_to_location",
    )

    ld.add_action(turtlesim_node)
    ld.add_action(spawn_turtle_node)
    ld.add_action(go_to_loc_node)

    return ld