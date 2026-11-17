"""Launch one isolated workcell and supervise Gazebo without a shell wrapper."""

import os
import shutil
import uuid

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    AppendEnvironmentVariable, DeclareLaunchArgument, ExecuteProcess,
    SetEnvironmentVariable, Shutdown,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('my_robot_description')
    with open(os.path.join(pkg, 'urdf', 'robot.urdf')) as urdf:
        robot_description = urdf.read()
    gz = shutil.which('gz')
    if gz is None:
        raise RuntimeError('Gazebo executable not found; source ROS Jazzy first')

    return LaunchDescription([
        DeclareLaunchArgument('pick_and_place', default_value='false'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument(
            'gazebo_partition', default_value=f'robot_prod_{uuid.uuid4().hex}',
            description='Gazebo transport partition shared only by this launch'),
        SetEnvironmentVariable('GZ_PARTITION', LaunchConfiguration('gazebo_partition')),
        SetEnvironmentVariable('GZ_IP', os.environ.get('GZ_IP', '127.0.0.1')),
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.dirname(pkg)),
        AppendEnvironmentVariable(
            'GZ_SIM_SYSTEM_PLUGIN_PATH', os.environ.get('LD_LIBRARY_PATH', '')),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        ),
        # Run server and GUI directly as separate processes. The upstream shell
        # wrapper can leave the actual server alive when launch terminates it.
        ExecuteProcess(
            cmd=['ruby', gz, 'sim', '-r', '-s', os.path.join(pkg, 'worlds', 'workcell.sdf'),
                 '--force-version', '8'],
            name='gazebo_server', output='screen', on_exit=Shutdown(),
        ),
        ExecuteProcess(
            cmd=['ruby', gz, 'sim', '-g', '--force-version', '8'],
            name='gazebo_gui', output='screen', condition=IfCondition(LaunchConfiguration('gui')),
            on_exit=Shutdown(),
        ),
        Node(
            package='ros_gz_bridge', executable='parameter_bridge', name='robot_gz_bridge',
            parameters=[{'config_file': os.path.join(pkg, 'config', 'gz_bridge.yaml')}],
            output='screen',
        ),
        Node(
            package='robot_motion',
            executable=PythonExpression([
                "'pick_place' if '", LaunchConfiguration('pick_and_place'),
                "' == 'true' else 'turnNode'",
            ]),
            name='turn_node', parameters=[{'use_sim_time': True}], output='screen',
        ),
        Node(
            package='monitor', executable='joint_monitor',
            parameters=[{'use_sim_time': True}], output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2',
            condition=IfCondition(LaunchConfiguration('gui')),
            arguments=['-d', os.path.join(pkg, 'config', 'display.rviz')],
            parameters=[{'use_sim_time': True}], output='screen',
        ),
    ])
