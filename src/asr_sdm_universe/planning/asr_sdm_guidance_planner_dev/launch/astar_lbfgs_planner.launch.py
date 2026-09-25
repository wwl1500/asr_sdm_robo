from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('asr_sdm_guidance_planner_dev')
    default_config = os.path.join(pkg_share, 'config', 'astar_lbfgs_planner.yaml')
    default_rviz_config = os.path.join(pkg_share, 'rviz', 'astar_esdf_planner.rviz')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Path to the A* + L-BFGS planner configuration file.'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to start RViz2 together with the planner.'
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_config,
        description='Path to the RViz2 configuration file.'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='world',
        description='Frame used by planner outputs and occupied_map marker. Keep it the same as RViz Fixed Frame.'
    )

    planner_node = Node(
        package='asr_sdm_guidance_planner_dev',
        executable='rviz_astar_lbfgs_planner',
        name='astar_lbfgs_path_planner',
        output='screen',
        parameters=[LaunchConfiguration('config'), {'frame_id': LaunchConfiguration('frame_id')}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        config_arg,
        use_rviz_arg,
        rviz_config_arg,
        frame_id_arg,
        planner_node,
        rviz_node,
    ])
