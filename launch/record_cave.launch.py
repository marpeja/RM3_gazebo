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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


from launch_ros.actions import Node


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([          
        
	DeclareLaunchArgument(
	    'use_sim_time',
	    default_value=use_sim_time,
	    description="Use sim clock or not"),

                
#        Node(
#            package='rm3_gazebo',
#            executable='oc_grid_lec.py',
#            name='ocGridLec',
#            parameters= [{'use_sim_time': use_sim_time}],
#            output='screen',
#        ),

	Node(
	    package='rm3_gazebo',
	    executable='follow_right_wall.py',
	    name='follow_right_wall',
	    parameters=[{'use_sim_time': use_sim_time}],
	    output='screen',
	),
    ])
