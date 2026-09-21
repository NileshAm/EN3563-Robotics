from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import shlex


def generate_launch_description():

    pkg = get_package_share_directory(
        'my_robot_description'
    )

    urdf_file = os.path.join(
        pkg,
        'urdf',
        'robot.urdf'
    )


    robot_description = open(
        urdf_file
    ).read()

    bridge_config = os.path.join(pkg, 'config', 'gz_bridge.yaml')
    rviz_config = os.path.join(pkg, 'config', 'display.rviz')


    return LaunchDescription([

        # Robot state publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[
                {
                    'robot_description':
                    robot_description,
                    'use_sim_time': True,
                }
            ]
        ),


        # Gazebo
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory(
                        'ros_gz_sim'
                    ),
                    'launch',
                    'gz_sim.launch.py'
                )
            ),
            launch_arguments={
                'gz_args': '-r ' + shlex.quote(
                    os.path.join(pkg, 'worlds', 'workcell.sdf')
                ),
                'on_exit_shutdown': 'true',
            }.items(),
        ),

        # ROS 2 -> Gazebo joint commands and Gazebo -> ROS 2 feedback.
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='robot_gz_bridge',
            parameters=[{'config_file': bridge_config}],
            output='screen',
        ),

        # Publish the target marker and the planned joint commands.
        Node(
            package='robot_motion',
            executable='turnNode',
            name='turn_node',
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

        # RViz follows /tf generated from Gazebo's bridged /joint_states.
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            parameters=[{'use_sim_time': True}],
            output='screen',
        ),

    ])
