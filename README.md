
MoveIt! commander ROS node
=====================================================================

This ROS node is used for giving orders to the Fontys Smart Wrist.

Dependencies
=====================================================================
[The robot model](https://github.com/fontyssmartwrist/sw_v3_description)

[The MoveIt! package](https://github.com/fontyssmartwrist/moveit_smart_wrist)

Install guide
=====================================================================
   1) Install MoveIt! kinetic with the following command

	sudo apt install ros-kinetic-moveit

   2) Install MoveIt! TRAC-IK plugin with the following command

	sudo apt-get install ros-kinetic-trac-ik-kinematics-plugin

   3) Clone this project and the dependencies to your catkin's workspace src folder
   4) Run catkin_make to build 
   
Documentation
=====================================================================
Documentation on how it works can be found in the [wiki](https://github.com/fontyssmartwrist/smart_movement/wiki).

Authors
=====================================================================
Aike van Alkemade 
Mahmoud Omar Ouali


