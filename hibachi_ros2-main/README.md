# hibachi_ros2
ROS2 packages for Hibachi skid steer robot

## Installing

This project is build and tested on Ubuntu 22.04 LTS with ROS 2 Humble LTS.

### Setup workspace
```
mkdir -p ~/workspaces/ros2_ws/src
cd ~/workspaces/ros2_ws/src
git clone https://github.com/gmsanchez/hibachi_ros2.git
```

### Clone serial submodule
```
cd ~/workspaces/ros2_ws/src/hibachi_ros2
git submodule init
git submodule update
```

### Clone extra repositories
```
cd ~/workspaces/ros2_ws/src
git clone -b ros2 https://github.com/ros-drivers/nmea_navsat_driver.git
git clone -b ros2_0_humble https://github.com/gmsanchez/bluespace_ai_xsens_ros_mti_driver.git
```

### Install dependencies
```
cd ~/workspaces/ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

### Install extra dependencies

These packages are dependencies, but for some reason are not installed automatically.

nmea_navsat_driver needs

```
sudo apt install python3-transforms3d
```

### Build and run
```
cd ~/workspaces/ros2_ws
colcon build
source install/setup.bash
ros2 launch hibachi_hardware hibachi_hardware_launch.py
```

## Thanks to

### ros2_control examples and documentation

* [ROS2 Docs: Writing a new hardware interface](https://control.ros.org/master/doc/ros2_control/hardware_interface/doc/writing_new_hardware_interface.html)
* [Mecanum Bot ROS2 package (foxy)](https://github.com/deborggraever/ros2-mecanum-bot)
* [Mecanum Bot ROS2 firmware](https://github.com/deborggraever/ros2-mecanum-bot-firmware)
* [Clearpath Robotics Husky (galactic-devel)](https://github.com/husky/husky/tree/galactic-devel)
* [Husarion Rosbot ROS2 package (humble)](https://github.com/husarion/rosbot_ros/tree/humble) 
* [Husarion Rosbot ROS2 hardware interface](https://github.com/husarion/rosbot_hardware_interfaces)
* [ros2_control Demos](https://github.com/ros-controls/ros2_control_demos)
* [STL files from Andrew-rw](https://github.com/Andrew-rw/different_parts_stl)

### Serial interface provided by

* [wjwwood/serial](https://github.com/wjwwood/serial/tree/ros2)
* [RoverRobotics-forks/serial-ros2](https://github.com/RoverRobotics-forks/serial-ros2)
* [RFRIEDM-Trimble/serial-ros2](https://github.com/RFRIEDM-Trimble/serial-ros2)