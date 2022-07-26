# ROBOMINER SIMULATOR (ROS2 Foxy + Gazebo 11)


![Gazebo simulator](/docs/logo_rm_horiz.png?raw=true "Title")
![Gazebo simulator](/docs/gazebo_screenshot_whiskers.png?raw=true "Title")

This repo contains a Gazebo11 + ROS2 Foxy simulator for the Robominer lab prototype RM3 designed at Taltech University.

Make sure to install the robominer package first:
https://github.com/robominers-eu/TALTECH_RM3_ROS_PACKAGES

You may have to add the installation prefix of robominer_msgs after colcon build robominer_msgs before the rest of the packages:

```
export CMAKE_PREFIX_PATH=...../dev_ws/install/robominer_msgs/share/robominer_msgs/cmake
```


# Install ros2_control dependencies from Foxy branch:

```
cd ~/dev_ws/src
git clone -b foxy https://github.com/ros-controls/ros2_control.git
git clone -b foxy https://github.com/ros-controls/ros2_controllers.git
git clone -b foxy https://github.com/ros-simulation/gazebo_ros2_control.git
```

## Install dependencies:
```
cd ~/dev_ws/
rosdep install --from-paths src --ignore-src -r -y
```

## Colcon build ros2_control packages:
```
colcon build --symlink-install --parallel-workers 3
```

(if you get an error related to gripper_controllers, then just delete the folder gripper_controllers from ~/dev_ws/src/ros2_controllers and “colcon build” again)

```
cd ~/dev_ws/
source install/setup.bash
```

# RM3 simulator installation:

## to clone via https:
```
cd ~/dev_ws/src
git clone https://github.com/marpeja/RM3_gazebo.git
```

## to clone via SSH:
```
git clone git@github.com:marpeja/RM3_gazebo.git
```

## Compile:

```
cd ~/dev_ws/
colcon build --symlink-install --parallel-workers 3
source install/setup.bash
```

## To launch the simulator:

For first time launch, First export the path of the models to Gazebo.

```
echo 'export GAZEBO_MODEL_PATH=~/dev_ws/src/RM3_gazebo/RM3_gazebo/models' >> ~/.bashrc

source ~/.bashrc
```
## Launching the simulator
```
ros2 launch rm3_gazebo spawn_RM3.launch.py
```

## To control with Keyboard:

After launching Gazebo, run in another terminal:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=cmd_vel_keyboard
```

## Worlds in this repo:
There are currently 4 world models:
  * empty_world: basic empty world with only ground and RM3 robot.
  * boxes_world: World with boxes in front of the robot to test whiskers collision behaviour.
  * terrain_world: World with an uneven terrain.
  * cave_world: World containing a mesh of the Belgium Cave in Han-sur-Lesse.

To select a world, you need to edit the "_world_" variable in file "_/config/simulation_parameters.yaml_".

## Setting simulation parameters:
We now use a YAML file to set the simulation parameters. We use the PyYAML library to read the file content on Python. To install PyYAML:
```
sudo pip install pyyaml
```

### Enabling / Disabling Whiskers on the robot:
To enable or disable the simulation of the whiskers on the robot, simply edit the file "_/config/simulation_parameters.yaml_", and set the parameters "_enableWhiskers_" to either "_enable_" or "_disable_".
### Enabling / Disabling RGB-D camera on the robot:
To enable or disable the simulation of the RGBD camera on the robot, simply edit the file "_/config/simulation_parameters.yaml_", and set the parameters "_enableCamera_" to either "_enable_" or "_disable_".

## RTABMAP for SLAM mapping using an rgb-d camera
We currently use RTABMAP for SLAM using the on-board camera (Realsense D455). To use it on the simulator also, you need to get the RTABMAP package for ROS2.

```
cd ~/dev_ws/src
git clone --branch foxy-devel https://github.com/introlab/rtabmap.git
git clone --branch foxy-devel https://github.com/introlab/rtabmap_ros.git
export MAKEFLAGS="-j2" # Can be ignored if you have a lot of RAM (>16GB)
cd ~/dev_ws
colcon build --symlink-install
```
To launch the SLAM nodes, run. This will launch RTABMAP in mapping mode:

```
ros2 launch rm3_gazebo RM3_RTABMAP.launch.py
```
To visualize the point cloud in Rviz2, you need to set the fixed frame to '_map_' instead of '_world_'

To launch RTABMAP in localization node:
```
ros2 launch rm3_gazebo RM3_RTABMAP.launch.py localization:=true
```

RTABMAP will create a folder (by default called .ros) where the info of the mapping session will be stored after rtabmap.db. This file is deleted every time the node is launched in Mapping mode, in order to store the new session. Several sessions have been already recorded to simplify the test of the package. You can simply copy the rtabmap.db you want to try from the ............ package and save it under the .ros folder mentioned. After that, launch the rtabmap node in localization node.


## Known bugs:
When running '_ros2 launch rm3_gazebo spawn_RM3.launch.py_', if you get an error saying '*.py' not found on the libexec directory. It means you need to make the python scripts executable. To do so:
```
sudo chmod +x ~/dev_ws/src/robominer_taltech/RM3_gazebo/scripts/*.py
```

## Nav2

### Install
```
pip install xacro
sudo apt install ros-foxy-slam-toolbox
sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup ros-foxy-turtlebot3*
```
```
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

mkdir -p dev_ws/src
cd dev_ws/src
git clone https://github.com/marpeja/RM3_nav.git
cd ..
rosdep update
rosdep install -y -r -q --from-paths src --ignore-src --rosdistro foxy 
colcon build --symlink-install
```
### Navigate

Launching navigation2:

```
ros2 launch nav_rm3 navigation2.launch.py
```


