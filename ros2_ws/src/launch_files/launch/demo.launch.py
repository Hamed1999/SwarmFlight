from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="py_package",
            executable="talker",
            name="talker_node",
            output="screen"),
             
        Node(
            package="py_package",
            executable="listener",
            name="listener_node",
            output="screen"
        )

    ])