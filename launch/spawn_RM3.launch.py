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
from launch.actions import ExecuteProcess, IncludeLaunchDescription, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch.conditions import IfCondition


from launch_ros.actions import Node

import xacro

import yaml


def generate_launch_description():

    simulation_parameters_yaml = os.path.join(
        get_package_share_directory('rm3_gazebo'),
        'config',
        'simulation_parameters.yaml'
        )

    with open(simulation_parameters_yaml, 'r') as file:
        # The FullLoader parameter handles the conversion from YAML
        # scalar values to Python the dictionary format
        sim_params = yaml.load(file, Loader=yaml.FullLoader)


    world_path = PathJoinSubstitution(
        [FindPackageShare("rm3_gazebo"), "worlds", sim_params["world"] + ".world"]
    )
    
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    steering_node = launch_ros.actions.Node(
        package='robominer_locomotion_control',
        executable='open_loop_steering',
        name='open_loop_steering',
        output='screen',
        parameters=[{'on_robot': False},
                    {'which_sim': 'gazebo'},
                    {'use_sim_time': use_sim_time}
                    ]
        )

    rm3_gazebo_path = os.path.join(
        get_package_share_directory('rm3_gazebo'))

    xacro_file = os.path.join(rm3_gazebo_path,
                              'urdf',
                              'RM3_robot.xacro.urdf')

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml(), 'ignore_timestamp': True, 'use_sim_time': use_sim_time}

    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'RM3',
                                   '-x', '-0.5',
                                   '-y', '0.0',
                                   '-z', '0.32',
                                   ],
                         
         		 parameters= [{'use_sim_time': True}],
                        output='screen')

    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen',
    )
    
    set_controller_use_sim_time = ExecuteProcess(
    	cmd=['ros2', 'param', 'set', '/controller_manager', 'use_sim_time', use_sim_time],
    	output='screen',
    ) 

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'velocity_controller'],
        output='screen',
    )

    enableWhiskers = LaunchConfiguration('enableWhiskers')
    if sim_params['sensors']['whiskers']['enable_whiskers'] == "enable":
        enableWhiskersValue = 'True'
    else:
        enableWhiskersValue = 'False'


    return LaunchDescription([
        DeclareLaunchArgument(
            'enableWhiskers',
            default_value = enableWhiskersValue,
            description='Whether to start Whisker related nodes'),
            
        
	DeclareLaunchArgument(
	    'use_sim_time',
	    default_value=use_sim_time,
	    description="Use sim clock or not"),

        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so',  '-s', 'libgazebo_ros_init.so', world_path],
            output='screen'
        ),

        Node(
            package='rm3_gazebo',
            executable='motor_to_body_vel.py',
            name='motor2bodyvel',
            parameters= [{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        Node(
            package='rm3_gazebo',
            executable='whisker_state_publisher.py',
            name='whiskerStates',
            output='screen',
            condition = IfCondition(enableWhiskers),
            parameters=[{'which_representation': "Spherical"},
                        {'number_of_arrays': 6},
                        {'use_sim_time': use_sim_time},
                        ]
        ),

        Node(
            package='rm3_gazebo',
            executable='whisker_state_visualizer.py',
            name='whiskerPointCloud',
            condition = IfCondition(enableWhiskers),  
            parameters= [{'use_sim_time': use_sim_time}],
            output='screen',
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_entity,
                on_exit=[set_controller_use_sim_time],
            )
        ),
        
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=set_controller_use_sim_time,
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
