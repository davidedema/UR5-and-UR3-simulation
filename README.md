# UR5-and-UR3-simulation
Simulation of two robotic arms (UR5 and UR3) with an OnRobot 2 finger soft gripper

## Install the project
The project was developed with ROS noetic and Gazebo. In order to use the project:
```
cd ros_ws/src
catkin_make install
source install/setup.bash
```
## Simulation
The simulation have two robotic arms: one UR5 and one UR3. They, of course, can be controlled indipendently with their topics. I've created 2 simple scripts for the UR5 and UR3 motion: `motionGroupPosController.py` for the UR5 and `motionGroupPosControllerCopy.py` for the UR3.

You can run it in this way:
```
roscd two_arms
cd scripts
python3 motionGroupPosController.py
``` 