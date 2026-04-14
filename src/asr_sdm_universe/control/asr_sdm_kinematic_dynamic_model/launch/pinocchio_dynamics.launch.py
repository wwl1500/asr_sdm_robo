"""Minimal launch file for the Pinocchio dynamics publisher.

This is intentionally lightweight so it can be composed into larger launch
graphs (e.g. RViz visualization) without duplicating configuration logic.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import math
import os


def generate_launch_description():
    nominal_radius = 0.06
    nominal_length = 0.3
    nominal_volume = math.pi * nominal_radius * nominal_radius * nominal_length

    default_urdf = os.path.join(
        get_package_share_directory("asr_sdm_description"),
        "urdf",
        "underwater_snakerobot.urdf",
    )

    parameters = {
        # Pinocchio loads URDFs directly from disk, so we point to the installed file.
        "robot_description_path": default_urdf,
        "publish_period_ms": 200,
        "use_free_flyer": True,
        "hydrodynamic_link_names": ["base_link", "link_2", "link_3", "link_4"],
        "fluid_density": 1000.0,
        "cd_n": 1.0,
        "cd_t": 0.05,
        "link_volumes": [nominal_volume] * 4,
        "link_radii": [nominal_radius] * 4,
        "link_lengths": [nominal_length] * 4,
        "added_mass_factors": [1.0] * 4,
    }

    dynamics_node = Node(
        package="asr_sdm_kinematic_dynamic_model",
        executable="pinocchio_dynamics_node",
        name="pinocchio_dynamics_node",
        output="screen",
        parameters=[parameters],
    )

    return LaunchDescription([dynamics_node])
