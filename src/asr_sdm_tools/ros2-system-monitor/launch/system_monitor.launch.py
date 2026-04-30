#!/usr/bin/env python3

#################################################################################
# Copyright 2009, Willow Garage, Inc.
# Copyright 2013 by Ralf Kaestner
# Copyright 2013 by Jerome Maye
# Copyright 2023 by kei1107
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the the copyright holder nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#################################################################################

import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.launch_description import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def _to_launch_bool(value):
    return "true" if value else "false"

def _coerce_bool(value):
    if isinstance(value, str):
        normalized = value.strip().lower()
        return normalized in {"1", "true", "yes", "on"}
    return bool(value)

def _load_enable_settings(config_path):
    defaults = {
        "enable_cpu_monitor": True,
        "enable_hdd_monitor": True,
        "enable_mem_monitor": True,
        "enable_net_monitor": True,
        "enable_ntp_monitor": False,
    }

    try:
        with open(config_path, "r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file) or {}
    except (OSError, yaml.YAMLError):
        return {key: _to_launch_bool(value) for key, value in defaults.items()}

    monitor_sections = {
        "enable_cpu_monitor": "cpu_monitor",
        "enable_hdd_monitor": "hdd_monitor",
        "enable_mem_monitor": "mem_monitor",
        "enable_net_monitor": "net_monitor",
        "enable_ntp_monitor": "ntp_monitor",
    }

    resolved = {}
    for key, monitor_name in monitor_sections.items():
        value = (
            config.get(monitor_name, {})
            .get("ros__parameters", {})
            .get(key, defaults[key])
        )
        resolved[key] = _to_launch_bool(_coerce_bool(value))

    return resolved

def _create_monitor_nodes(context):
    config_file = LaunchConfiguration("system_monitor_config_file").perform(context)
    enable_settings = _load_enable_settings(config_file)

    cpu_monitor = Node(
        package="ros2_system_monitor",
        executable="cpu_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(enable_settings["enable_cpu_monitor"]),
    )
    hdd_monitor = Node(
        package="ros2_system_monitor",
        executable="hdd_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(enable_settings["enable_hdd_monitor"]),
    )
    mem_monitor = Node(
        package="ros2_system_monitor",
        executable="mem_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(enable_settings["enable_mem_monitor"]),
    )
    net_monitor = Node(
        package="ros2_system_monitor",
        executable="net_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(enable_settings["enable_net_monitor"]),
    )
    ntp_monitor = Node(
        package="ros2_system_monitor",
        executable="ntp_monitor_node.py",
        parameters=[LaunchConfiguration("system_monitor_config_file")],
        respawn=True,
        emulate_tty=True,
        condition=IfCondition(enable_settings["enable_ntp_monitor"]),
    )

    return [cpu_monitor, hdd_monitor, mem_monitor, net_monitor, ntp_monitor]

def generate_launch_description():
    config_file_default = os.path.join(
        get_package_share_directory("ros2_system_monitor"),
        "config",
        "system_monitor.yaml",
    )
    declare_config_file = DeclareLaunchArgument(
        name="system_monitor_config_file",
        default_value=config_file_default,
        description="system monitor config file path",
    )
    # create the launch description
    ld = LaunchDescription()

    # declare the launch options
    ld.add_action(declare_config_file)
    ld.add_action(OpaqueFunction(function=_create_monitor_nodes))

    return ld
