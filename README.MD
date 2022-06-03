# ROBOMINER SIMULATOR (ROS2 Foxy + Gazebo 11)


![Gazebo simulator](/docs/logo_rm_horiz.png?raw=true "Title")
![Gazebo simulator](/docs/gazebo_screenshot.png?raw=true "Title")

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
git clone https://github.com/walidremmas/robominer_taltech.git
```

## to clone via SSH:
```
git clone git@github.com:walidremmas/robominer_taltech.git
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
echo 'export GAZEBO_MODEL_PATH=~/dev_ws/src/robominer_taltech/RM3_gazebo/models' >> ~/.bashrc

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
