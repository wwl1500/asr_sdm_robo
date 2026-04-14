"""Launch file that visualizes the robot model in RViz2 together with the
Pinocchio-based dynamics publisher.

Usage:
  ros2 launch asr_sdm_kinematic_dynamic_model dynamics_rviz.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # ── Paths ──────────────────────────────────────────────────────────
    description_pkg_share = FindPackageShare("asr_sdm_description")
    dynamics_pkg_share = FindPackageShare("asr_sdm_kinematic_dynamic_model")

    default_urdf = os.path.join(
        get_package_share_directory("asr_sdm_description"),
        "urdf",
        "underwater_snakerobot.urdf",
    )

    default_rviz = PathJoinSubstitution(
        [dynamics_pkg_share, "launch", "dynamics_rviz.rviz"]
    )

    # ── Launch arguments ───────────────────────────────────────────────
    urdf_arg = DeclareLaunchArgument(
        "urdf_path",
        default_value=default_urdf,
        description="Absolute path to the URDF file.",
    )

    # ── Read URDF content ──────────────────────────────────────────────
    with open(default_urdf, "r") as f:
        robot_description_content = f.read()

    # ── robot_state_publisher ──────────────────────────────────────────
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description_content}],
    )

    # ── joint_state_publisher_gui (slider UI to move joints) ──────────
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        output="screen",
    )

    # ── Pinocchio dynamics node ────────────────────────────────────────
    dynamics_node = Node(
        package="asr_sdm_kinematic_dynamic_model",
        executable="pinocchio_dynamics_node",
        name="pinocchio_dynamics_node",
        output="screen",
        parameters=[
            {
                "robot_description_path": default_urdf,
                "publish_period_ms": 200,
                "use_free_flyer": True,
            }
        ],
    )

    # ── RViz2 ──────────────────────────────────────────────────────────
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", default_rviz],
    )

    return LaunchDescription(
        [
            urdf_arg,
            robot_state_publisher_node,
            joint_state_publisher_gui_node,
            dynamics_node,
            rviz_node,
        ]
    )
