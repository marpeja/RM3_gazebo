# ROBOMINER SIMULATOR (ROS2 + Gazebo 11)

This repo contains a gazebo+ROS2 simulator for the Robominer lab prototype propose by Taltech University.

# Clone this repo within your ROS2 dev_ws folder:
```
cd ~/dev_ws/src
```

# to clone via https:
```
git clone https://github.com/walidremmas/robominer_taltech.git
```

# to clone via SSH:
```
git clone git@github.com:walidremmas/robominer_taltech.git
```

# Compile:

```
cd ~/dev_ws/
colcon build
source install/setup.bash
```

# To launch the simulator:

```
ros2 launch robominer_gazebo gazebo.launch.py
```

If for some reason, Gazebo doesn't recognize the path to the robot's model, then add it manuallly (replace gazebo-11 with the version you are using):

```
sudo mkdir /usr/share/gazebo-11/models/robominer_description

sudo cp ~/dev_ws/src/robominer_taltech/robominer_description/model.config /usr/share/gazebo-11/models/robominer_description/

sudo cp -a ~/dev_ws/src/robominer_taltech/robominer_description/urdf/. /usr/share/gazebo-11/models/robominer_description/urdf
```

# To control with Keyboard:

After launching Gazebo, run in another terminal:

```
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```


