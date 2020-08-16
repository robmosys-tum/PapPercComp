## Fruit Detection Manipulation

## Description
This a ros package that manipulates a robot arm to move towards detections of infected fruit. A dummy fruit infection detector publisher is used for integration.
## Usage
* build with `catkin_make`
* source: `. devel/setup.bash`
* Launch two terminal windows and type: `roslaunch robot_move main.launch` in the first and `rosrun robot_move talker` in the second.
Rviz should open and you will see in the talker's terminal that multiple messages are being published.  Only the message containing a disease detection with a high score makes the robot move.