# Team 2:
Schnell Scott
Contreras Kurt
Mulrooney Brian
Bolton Aaron
# Launch Instructions
To launch use "roslaunch automatic_emergency_braking_t2 aeb.launch control:=0"
Control = 0 for TTC control
Control = 1 for Distance Control
# bootcamp-assignments
Please install the dependencies mentioned below, before building this repository.

### Dependencies:
Use `sudo apt install` to install the first 7 dependencies and then follow the specific instructions provided.
* ros-noetic-laser-proc*
* ros-noetic-urg-c
* ros-noetic-ros-control*
* ros-noetic-gazebo-ros-control
* ros-noetic-ackermann-msgs
* ros-noetic-joy
* ros-noetic-pid
* **OpenCV**:
  To install OpenCV use the command `sudo apt install libopencv-dev python3-opencv`
* To install the `driver-base` package follow these instructions:
  * Move into the `src` folder of your workspace (You can install this in the `ros_ws/src` as it's a dependency & you won't be pushing this to Github).
  * Clone the repository by entering `git clone https://github.com/ros-drivers/driver_common.git`
  * Enter `catkin_make` at the root of your repository.
* **QT4**: Install Qt4 (Ubuntu 20.04 comes with Qt5 by default) by following these instructions:
  * ```
    sudo add-apt-repository ppa:gezakovacs/ppa
    sudo apt update
    sudo apt install qt4-default
    ```

This repository contains skeleton code for some of the assignments in the SAE Robotics Bootcamp Course. 

The following assignments have been provided with skeleton code:

- Week 4: Automatic Emergency Braking
- Week 5: Wall Following
- Week 5: Turtlebot3 Line Following
- Week 6: Autonomous Lane Keeping
- Week 7: Behavior Cloning Line Following
- Week 7: Object Recognition

Individual instructions for the completion of these assignments are provided in the individual folders.

