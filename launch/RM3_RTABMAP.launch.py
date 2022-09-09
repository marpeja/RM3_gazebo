#!/usr/bin/python3
"""
RM3 SLAM using RTABMAP launch file.

@author: Walid Remmas
@contact: walid.remmas@ŧaltech.ee
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node

def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time')
    qos = LaunchConfiguration('qos')
    localization = LaunchConfiguration('localization')
    continue_slam = LaunchConfiguration('continue_slam')

    parameters={
          'frame_id':'base_link',
          'use_sim_time':use_sim_time,
          'subscribe_depth':True,
          'use_action_for_goal':False,
          'qos_image':qos,
          'qos_imu':qos,
          'Reg/Force3DoF':'true',
          'Optimizer/GravitySigma':'0', # Disable imu constraints (we are already in 2D),
          'Grid/MaxGroundHeight':'0.05',
          'GridGlobal/FootprintRadius': '0.4',
          'Grid/MaxObstacleHeight': '1.5',
          'RGBD/OptimizeMaxError': '4.0',
          'Vis/MinInliers': '20'
    }

    remappings_rtabmap=[
          ('rgb/image', '/camera/image_raw'),
          ('rgb/camera_info', '/camera/camera_info'),
          ('depth/image', '/camera/depth/image_raw'),
          ('odom', '/odom/unfiltered')]
          
    remappings_pc_to_ls=[
    	  ('depth', '/camera/depth/image_raw'),
    	  ('depth_camera_info', '/camera/depth/camera_info')]

    return LaunchDescription([

        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='Use simulation (Gazebo) clock if true'),

        DeclareLaunchArgument(
            'qos', default_value='2',
            description='QoS used for input sensor topics'),

        DeclareLaunchArgument(
            'localization', default_value='false',
            description='Launch in localization mode.'),
        
        DeclareLaunchArgument(
            'continue_slam', default_value='false',
            description='Launch in localization mode.'),

        # Nodes to launch

        # SLAM mode:
        Node(
            condition=UnlessCondition(localization),
            package='rtabmap_ros', executable='rtabmap', name='rtabmap_slam', output='screen',
            parameters=[parameters],
            remappings=remappings_rtabmap,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
       # Continue SLAM
        Node(
            condition=IfCondition(continue_slam),
            package='rtabmap_ros', executable='rtabmap', name='rtabmap_slam', output='screen',
            parameters=[parameters],
            remappings=remappings_rtabmap),
            
        # Localization mode:
        Node(
            condition=IfCondition(localization),
            package='rtabmap_ros', executable='rtabmap', name='rtabmap_localization', output='screen',
            parameters=[parameters,
              {'Mem/IncrementalMemory':'False',
               'Mem/InitWMWithAllNodes':'True'}],
            remappings=remappings_rtabmap),

        Node(
            package='rtabmap_ros', executable='rtabmapviz', name='rtabmapviz', output='screen',
            parameters=[parameters],
            remappings=remappings_rtabmap),
            
        Node(
            package='depthimage_to_laserscan', 
            executable='depthimage_to_laserscan_node',
            name='depthimage_to_laserscan_node', 
            output='screen',
            parameters=[{'output_frame':'realsense_camera',
                         'range_min': 0.05,
                         'scan_time': 0.016666,
                         'use_sim_time':use_sim_time}],
            remappings=remappings_pc_to_ls),
       
    ])
