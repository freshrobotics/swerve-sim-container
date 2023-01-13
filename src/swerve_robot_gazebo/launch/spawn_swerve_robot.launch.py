import os
from typing import Final, List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Packages
SWERVE_ROBOT_GAZEBO_PKG: Final[str] = "swerve_robot_gazebo"
SWERVE_ROBOT_DESCRIPTION_PKG: Final[str] = "swerve_robot_description"
SWERVE_CONTROLLER_PKG: Final[str] = "swerve_controller"

# File Names
DESCRIPTON_LAUNCH_FILE: Final[str] = "description.launch.py"
CONTROL_LAUNCH_FILE: Final[str] = "swerve_control_odom.launch.py"

# Directory Names
LAUNCH_DIR: Final[str] = "launch"

# File Paths
description_launch_path = os.path.join(
    get_package_share_directory(SWERVE_ROBOT_DESCRIPTION_PKG),
    LAUNCH_DIR,
    DESCRIPTON_LAUNCH_FILE,
)
swerve_controller_launch_path = os.path.join(
    get_package_share_directory(SWERVE_CONTROLLER_PKG), LAUNCH_DIR, CONTROL_LAUNCH_FILE
)

# Launch Arguments
ARGUMENTS: Final[List[DeclareLaunchArgument]] = []


def generate_launch_description() -> LaunchDescription:
    """
    1. Launch description launch file
    2. Load all joint state controller and 8 Gazebo joint controllers
    3. Launch control and odom nodes

    Returns:
        LaunchDescription
    """

    # Include description launch file
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([description_launch_path]),
        launch_arguments={"sim": "true"}.items(),
    )

    # Spawn robot
    spawn_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        arguments=[
            "-topic",
            "robot_description",
            "-entity",
            "swerve_robot",
            "-z",
            "0.35",
        ],
        output="screen",
    )

    # Joint State Controller
    load_joint_state_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        output="screen",
    )

    # Wheel Rotation Controllers
    load_fr_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_front_right",
        ],
        output="screen",
    )

    load_fl_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_front_left",
        ],
        output="screen",
    )

    load_rl_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_rear_left",
        ],
        output="screen",
    )

    load_rr_wheel_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "wheel_rear_right",
        ],
        output="screen",
    )

    # Wheel Steering Controllers
    load_fr_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "steering_front_right",
        ],
        output="screen",
    )

    load_fl_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "steering_front_left",
        ],
        output="screen",
    )

    load_rl_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "steering_rear_left",
        ],
        output="screen",
    )

    load_rr_steering_controller = ExecuteProcess(
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "steering_rear_right",
        ],
        output="screen",
    )

    eh_joint_state_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_controller],
        )
    )

    eh_fr_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_controller,
            on_exit=[load_fr_steering_controller],
        )
    )

    eh_fl_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fr_steering_controller,
            on_exit=[load_fl_steering_controller],
        )
    )

    eh_rl_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fl_steering_controller,
            on_exit=[load_rl_steering_controller],
        )
    )

    eh_rr_steer_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_rl_steering_controller,
            on_exit=[load_rr_steering_controller],
        )
    )

    eh_fr_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_rr_steering_controller,
            on_exit=[load_fr_wheel_controller],
        )
    )

    eh_fl_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fr_wheel_controller,
            on_exit=[load_fl_wheel_controller],
        )
    )

    eh_rl_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_fl_wheel_controller,
            on_exit=[load_rl_wheel_controller],
        )
    )

    eh_rr_wheel_ctrl = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_rl_wheel_controller,
            on_exit=[load_rr_wheel_controller],
        )
    )

    swerve_control_odom_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([swerve_controller_launch_path]),
    )

    ld = LaunchDescription(ARGUMENTS)
    ld.add_action(description_launch)
    ld.add_action(spawn_entity)
    ld.add_action(eh_joint_state_ctrl)
    ld.add_action(eh_fr_steer_ctrl)
    ld.add_action(eh_fl_steer_ctrl)
    ld.add_action(eh_rl_steer_ctrl)
    ld.add_action(eh_rr_steer_ctrl)
    ld.add_action(eh_fr_wheel_ctrl)
    ld.add_action(eh_fl_wheel_ctrl)
    ld.add_action(eh_rl_wheel_ctrl)
    ld.add_action(eh_rr_wheel_ctrl)
    ld.add_action(swerve_control_odom_launch)

    return ld
