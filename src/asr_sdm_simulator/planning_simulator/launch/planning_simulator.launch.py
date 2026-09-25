#!/usr/bin/env python3
"""Launch the planning simulator stack.

Node parameters live in config/planning_simulator.yaml.

    ros2 launch planning_simulator planning_simulator.launch.py
    ros2 launch planning_simulator planning_simulator.launch.py teleop:=enable
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _enabled(flag: LaunchConfiguration) -> IfCondition:
    return IfCondition(PythonExpression(["'", flag, "' == 'enable'"]))


def generate_launch_description():
    pkg_share = get_package_share_directory("planning_simulator")
    rviz_config = os.path.join(pkg_share, "config", "rviz.rviz")
    map_generator_config = os.path.join(
        get_package_share_directory("asr_sdm_map_generator"),
        "config", "asr_sdm_map_generator.yaml")

    robot_model = LaunchConfiguration("robot_model")
    odom_source = LaunchConfiguration("odom_source")
    control = LaunchConfiguration("control")
    teleop = LaunchConfiguration("teleop")
    planning = LaunchConfiguration("planning")
    log_collector = LaunchConfiguration("log_collector")

    sim_odom = "/control/asr_sdm/odom"
    vins_odom = "/localization/video_inertial_navigation_systems/odometry"

    robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value="asr_sdm",
        description="Robot model package; must provide launch/<name>_description.launch.py")
    odom_source_arg = DeclareLaunchArgument(
        "odom_source",
        default_value="auto",
        choices=["auto", "control", "vins"],
        description="Odometry that drives the robot model pose")
    control_arg = DeclareLaunchArgument(
        "control",
        default_value="enable",
        choices=["enable", "disable"],
        description="Start asr_sdm_control_manager kinematic controller")
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="disable",
        choices=["enable", "disable"],
        description="Start asr_sdm_teleop gamepad teleop chain")
    planning_arg = DeclareLaunchArgument(
        "planning",
        default_value="disable",
        choices=["enable", "disable"],
        description="Start asr_sdm_planning_manager planning chain")
    log_collector_arg = DeclareLaunchArgument(
        "log_collector",
        default_value="enable",
        choices=["enable", "disable"],
        description="Start asr_sdm_log_collector (merged rotating logs)")

    random_map_sensing = Node(
        package="asr_sdm_map_generator",
        executable="random_forest_test",
        name="random_map_sensing",
        output="screen",
        emulate_tty=True,
        parameters=[
            map_generator_config,
            {
                "init_state_x": -5.0,
                "init_state_y": 0.0,
                "sensing.enable_click_map": True,
                "topics.add_static_obstacle":
                    "/simulator/planning_simulator/add_static_obstacle",
            },
        ],
        remappings=[("odometry", sim_odom)],
    )

    odom_visualization = Node(
        package="odom_visualization",
        executable="odom_visualization",
        name="odom_visualization_ukf",
        output="screen",
        parameters=[{
            "color.a": 0.8,
            "color.r": 1.0,
            "color.g": 0.0,
            "color.b": 0.0,
            "covariance_scale": 100.0,
            "tf45": True,
            "odom_topics": [sim_odom, vins_odom],
        }],
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'auto'"])),
    )
    odom_visualization_control = Node(
        package="odom_visualization",
        executable="odom_visualization",
        name="odom_visualization_ukf",
        output="screen",
        parameters=[{
            "color.a": 0.8,
            "color.r": 1.0,
            "color.g": 0.0,
            "color.b": 0.0,
            "covariance_scale": 100.0,
            "tf45": True,
            "odom_topics": [sim_odom],
        }],
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'control'"])),
    )
    odom_visualization_vins = Node(
        package="odom_visualization",
        executable="odom_visualization",
        name="odom_visualization_ukf",
        output="screen",
        parameters=[{
            "color.a": 0.8,
            "color.r": 1.0,
            "color.g": 0.0,
            "color.b": 0.0,
            "covariance_scale": 100.0,
            "tf45": True,
            "odom_topics": [vins_odom],
        }],
        condition=IfCondition(PythonExpression(["'", odom_source, "' == 'vins'"])),
    )

    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare(robot_model),
                "launch",
            ]),
            "/",
            robot_model,
            "_description.launch.py",
        ]),
    )

    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_control_manager"),
                "launch", "asr_sdm_control_manager.launch.py")),
        condition=_enabled(control),
    )
    teleop_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_teleop"),
                "launch", "asr_sdm_teleop.launch.py")),
        condition=_enabled(teleop),
    )
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_planning_manager"),
                "launch", "asr_sdm_planning_manager.launch.py")),
        launch_arguments={"odom_topic": sim_odom}.items(),
        condition=_enabled(planning),
    )
    log_collector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("asr_sdm_log_collector"),
                "launch", "asr_sdm_log_collector.launch.py")),
        condition=_enabled(log_collector),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz",
        arguments=["-d", rviz_config],
    )

    return LaunchDescription([
        robot_model_arg,
        odom_source_arg,
        control_arg,
        teleop_arg,
        planning_arg,
        log_collector_arg,
        random_map_sensing,
        odom_visualization,
        odom_visualization_control,
        odom_visualization_vins,
        robot_description,
        control_launch,
        teleop_launch,
        planning_launch,
        log_collector_launch,
        rviz,
    ])
