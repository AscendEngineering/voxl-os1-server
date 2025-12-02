import os
import launch
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription(
        [DeclareLaunchArgument(name='log_level', default_value='info')])

    # Creating the path to the yaml files for ros parameters
    mavsdk_config = os.path.join("src", "mavsdk_drone", "parameters",
                              "mavsdk_drone_parameters.yaml")

    os1_config = os.path.join("src", "os1", "parameters",
                                "os1_parameters.yaml")

    # Nodes to launch on boot of the ros2 instance
    nodes = [
        Node(
            package="mavsdk_drone",
            executable="mavsdk_drone",
            parameters=[mavsdk_config],
            output="screen",
            emulate_tty=True,
        ),
        Node(
            package="os1",
            executable="os1",
            parameters=[os1_config],
            output="screen",
            emulate_tty=True,
        ),
    ]

    # Add nodes
    for node in nodes:
        ld.add_action(node)

    return ld
