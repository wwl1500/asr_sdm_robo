from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, ExecuteProcess
from launch.actions import RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    package_share = get_package_share_directory('asr_sdm_kinematic_dynamic_model')
    robot_share = get_package_share_directory('asr_sdm')
    urdf_path = Path(robot_share) / 'urdf/generated/asr_sdm_segments_4.urdf'
    rviz_path = Path(package_share) / 'rviz/underwater_simulator.rviz'
    robot_description = urdf_path.read_text()
    panel = (
        Path(package_share).parent.parent
        / 'lib/asr_sdm_kinematic_dynamic_model/underwater_control_panel.py'
    )

    simulator_node = Node(
        package='asr_sdm_kinematic_dynamic_model',
        executable='underwater_simulator_node',
        name='underwater_simulator_node',
        output='screen',
        parameters=[{
            'dt': ParameterValue(LaunchConfiguration('dt'), value_type=float),
            'duration': ParameterValue(LaunchConfiguration('duration'), value_type=float),
            'run_forever': ParameterValue(
                LaunchConfiguration('run_forever'), value_type=bool),
            'auto_shutdown': ParameterValue(
                LaunchConfiguration('auto_shutdown'), value_type=bool),
            'csv_path': LaunchConfiguration('csv_path'),
            'root_frame': LaunchConfiguration('root_frame'),
            'z_to_x_map': ParameterValue(
                LaunchConfiguration('z_to_x_map'), value_type=bool),
            'integrator': LaunchConfiguration('integrator'),
            'force_scale': ParameterValue(
                LaunchConfiguration('force_scale'), value_type=float),
            'max_trajectory_points': ParameterValue(
                LaunchConfiguration('max_trajectory_points'), value_type=int),
            'joint_viscous_damping': ParameterValue(
                LaunchConfiguration('joint_viscous_damping'), value_type=str),
            'joint_coulomb_friction': ParameterValue(
                LaunchConfiguration('joint_coulomb_friction'), value_type=str),
            'joint_friction_velocity_scale': ParameterValue(
                LaunchConfiguration('joint_friction_velocity_scale'), value_type=float),
            'joint_limit_stiffness': ParameterValue(
                LaunchConfiguration('joint_limit_stiffness'), value_type=float),
            'joint_limit_damping': ParameterValue(
                LaunchConfiguration('joint_limit_damping'), value_type=float),
            'joint_limit': ParameterValue(
                LaunchConfiguration('joint_limit'), value_type=float),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('dt', default_value='0.005'),
        DeclareLaunchArgument('duration', default_value='60.0'),
        DeclareLaunchArgument('run_forever', default_value='false'),
        DeclareLaunchArgument('auto_shutdown', default_value='false'),
        DeclareLaunchArgument('csv_path', default_value='underwater_simulation.csv'),
        DeclareLaunchArgument('with_rviz', default_value='true'),
        DeclareLaunchArgument('with_control_panel', default_value='false'),
        DeclareLaunchArgument('rvizconfig', default_value=str(rviz_path)),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('z_to_x_map', default_value='false'),
        DeclareLaunchArgument('root_frame', default_value='base'),
        DeclareLaunchArgument('integrator', default_value='semi_implicit_euler'),
        DeclareLaunchArgument('force_scale', default_value='0.02'),
        DeclareLaunchArgument('max_trajectory_points', default_value='3000'),
        # Joint dynamics parameters (6-DOF vectors as strings, scalars as floats)
        DeclareLaunchArgument(
            'joint_viscous_damping',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Joint viscous damping coefficients (Nm·s/rad) for 6 joints'),
        DeclareLaunchArgument(
            'joint_coulomb_friction',
            default_value='[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            description='Joint Coulomb friction coefficients (Nm) for 6 joints'),
        DeclareLaunchArgument(
            'joint_friction_velocity_scale',
            default_value='0.001',
            description='Velocity scale for Coulomb friction regularization (rad/s)'),
        DeclareLaunchArgument(
            'joint_limit_stiffness',
            default_value='0.0',
            description='Joint limit spring stiffness (Nm/rad), 0.0 disables limit forces'),
        DeclareLaunchArgument(
            'joint_limit_damping',
            default_value='0.0',
            description='Joint limit damping coefficient (Nm·s/rad)'),
        DeclareLaunchArgument(
            'joint_limit',
            default_value='1.5707963267948966',
            description='Joint travel range limit (rad), default ±π/2'),
        simulator_node,
        RegisterEventHandler(OnProcessExit(
            target_action=simulator_node,
            on_exit=[EmitEvent(event=Shutdown(reason='underwater simulator exited'))],
        )),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='underwater_robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
            remappings=[('joint_states', '/underwater_simulator/joint_states')],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='underwater_rviz2',
            output='screen',
            arguments=['-d', LaunchConfiguration('rvizconfig')],
            condition=IfCondition(LaunchConfiguration('with_rviz')),
        ),
        ExecuteProcess(
            condition=IfCondition(LaunchConfiguration('with_control_panel')),
            cmd=[
                FindExecutable(name='python3'),
                str(panel),
                '--node', '/underwater_simulator_node',
            ],
            output='screen',
        ),
    ])
