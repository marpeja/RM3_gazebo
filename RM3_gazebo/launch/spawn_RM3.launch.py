#!/usr/bin/python3
"""
RM3 simulation launch file.

@author: Walid Remmas
@contact: walid.remmas@ŧaltech.ee
"""

import os

from ament_index_python.packages import get_package_share_directory
import launch_ros

from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node

import xacro


def generate_launch_description():

    world_path = PathJoinSubstitution(
        [FindPackageShare("rm3_gazebo"), "worlds", "cave_world.world"]
    )
    steering_node = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='open_loop_steering',
        name='open_loop_steering',
        output='screen',
        parameters=[{'on_robot': False},
                    {'which_sim': 'gazebo'}]
        )

    rm3_gazebo_path = os.path.join(
        get_package_share_directory('rm3_gazebo'))

    xacro_file = os.path.join(rm3_gazebo_path,
                              'urdf',
                              'RM3_robot.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'RM3',
                                   '-x', '0.0',
                                   '-y', '0.0',
                                   '-z', '0.05',
                                   ],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'velocity_controller'],
        output='screen'
    )

    return LaunchDescription([
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', world_path],
            output='screen'
        ),
        Node(
            package='rm3_gazebo',
            executable='motor_to_body_vel.py',
            name='motor2bodyvel',
            output='screen'
        ),

        Node(
            package='rm3_gazebo',
            executable='whisker_state_publisher.py',
            name='whiskerStates',
            output='screen',
            parameters=[{'which_representation': "Cartesian"}]
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[load_joint_state_controller],
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller,
                on_exit=[load_joint_trajectory_controller],
            )
        ),
        steering_node,
        node_robot_state_publisher,
        spawn_entity,

    ])
