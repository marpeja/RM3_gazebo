import os

import launch
# import launch_ros

from launch.launch_description_sources import PythonLaunchDescriptionSource

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    joy_kinematics = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_bringup'),
                'launch',
                'sim_open_loop_steering_joy.launch.py')
        ))
    simulation = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('robominer_gazebo'),
                'launch',
                'gazebo.launch.py')
        ))

    return launch.LaunchDescription([
        joy_kinematics,
        simulation
        ])