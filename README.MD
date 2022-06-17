# ROBOMINER SIMULATOR (ROS2 Foxy + Gazebo 11)


![Gazebo simulator](/docs/logo_rm_horiz.png?raw=true "Title")
![Gazebo simulator](/docs/gazebo_screenshot_whiskers.png?raw=true "Title")

This repo contains a Gazebo11 + ROS2 Foxy simulator for the Robominer lab prototype RM3 designed at Taltech University.

Make sure to install the robominer package first:
https://github.com/robominers-eu/TALTECH_RM3_ROS_PACKAGES

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
git clone https://github.com/robominers-eu/RM3_gazebo.git
```

## to clone via SSH:
```
git clone git@github.com:robominers-eu/RM3_gazebo.git
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
There are currently 3 world models:
  * empty_world: basic empty world with only ground and RM3 robot.
  * boxes_world: World with a box in front of the robot to test whiskers collision behaviour.
  * terrain_world: World with an uneven terrain.

To select a world, you need to edit the "_world_path_" variable in file "_/launch/spawn_RM3.launch.py_". colcon build the package for changes to take effect.

## Enabling / Disabling Whiskers on the robot:
To enable or disable the simulation of the whiskers on the robot, simply edit the file "_/urdf/RM3_robot.xacro.urdf_", and set the xacro:property "_enableWhiskers_" to either "_enable_" or "_disable_". colcon build the package for changes to take effect.

## Enabling / Disabling RGB-D camera on the robot:
if you want to use the camera make sure to enable it on the file "_/urdf/RM3_robot.xacro.urdf_", and set the xacro:property "_enableCamera_" to "_enable_". colcon build the package for changes to take effect.
