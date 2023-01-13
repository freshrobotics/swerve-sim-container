import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

# Packages
DESCRIPTION_PKG: Final[str] = "swerve_robot_description"

# Directories
URDF_DIR: Final[str] = "urdf"

# Files
ROBOT_XACRO_FILE: Final[str] = "swerve_robot.xacro"

# File Paths
description_path = os.path.join(
    get_package_share_directory(DESCRIPTION_PKG), URDF_DIR, ROBOT_XACRO_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = [
    DeclareLaunchArgument(
        name="sim",
        default_value="false",
        choices=["true", "false"],
        description="Enable simulation control and sensor plugins",
    ),
]


def generate_launch_description() -> LaunchDescription:
    """
    1. Parse robot xacro file
    2. Launch robot state publisher for /robot_description and /tf

    Launch Args:
    sim: false(default)
        - Loads gazebo control and sensor plugins
    """

    sim_flag = LaunchConfiguration("sim")

    robot_description = {
        "robot_description": Command(
            [
                "xacro --verbosity 0 ",
                description_path,
                " sim:=",
                sim_flag,
            ]
        )
    }

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(robot_state_publisher_node)

    return ld
