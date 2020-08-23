## Fruit Detection Manipulation

# Instatllation
* The project works with ros melodic and moveit 1
* Ros melodic can be installed from here http://wiki.ros.org/melodic/Installation
* Moveit 1 binary installation can be found here https://moveit.ros.org/install/
* Make sure Eigen3 is installed `sudo apt-get install libeigen3-dev
* Make sure boost is installed `sudo apt install libboost-all-dev`
* Clone the repository
* Build the project with `catkin_make`

## Description
This a ros package and its dependencies. 
The package robot_move manipulates a robot arm to move towards detections of infected fruit. A dummy fruit infection detector publisher is used for integration.

## Usage
* source: `. devel/setup.bash`
* Launch two terminal windows and type: `roslaunch robot_move main.launch` in the first and `rosrun robot_move talker` in the second.
Rviz should open and you will see in the talker's terminal that multiple messages are being published.  Only the message containing a disease detection with a high score makes the robot move.