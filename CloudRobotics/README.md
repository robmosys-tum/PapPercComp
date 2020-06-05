# Interfacing Knowledge Base from Papyrus4Robotics

## Modules

|  Name  |  Description  |  Installation Link  | Version |
|:---:|:---:|:---:|:---:|
| ROS | The Robot Operating System (ROS) is a set of software libraries and tools for building robot applications | [Website](http://wiki.ros.org/melodic/Installation/Ubuntu) | --- (state 07.05.2020)|
| ROS2 | Version 2 of ROS, the goal of the ROS 2 project is to leverage what is great about ROS 1 and what isn't | [Website](https://index.ros.org/doc/ros2/Installation/Eloquent/) | eloquent |
| rosbridge | ROS 2 package that provides bidirectional communication between ROS 1 and ROS 2 |  *sudo apt install ros-eloquent-ros1-bridge* | eloquent |
| KnowRob | KnowRob is a knowledge base for robots | [Forked Repository](https://github.com/jkabalar/knowrob/tree/melodic) | master |
| Papyrus4Robotics (optional) | Is a model-based design/graphical editing tool for robotic applications | [Website](https://www.eclipse.org/papyrus/components/robotics/) | --- (state 07.05.2020)|

## Installation (KnowRob)

The following instructions were tested on Ubuntu 18.04. Installation from source using the 'catkin' buildsystem is recommendended.

The [Installation Instructions](http://knowrob.org/installation) are adapted as following

```bash
rosdep update
cd ~/catkin_ws/src
wstool merge https://raw.github.com/jkabalar/knowrob/master/rosinstall/knowrob-base.rosinstall
wstool update
rosdep install --ignore-src --from-paths .
cd ~/catkin_ws
catkin_make
```

Also make sure to initialize the [Workspace](http://knowrob.org/installation/workspace/) afterwards.

*NOTE: the [Forked Repository](https://github.com/jkabalar/knowrob/tree/melodic) contains small adaptions and fixes, e.g. adding of the json_prolog (original from kinetics branch) package* 

## Running Instructions (Rosbridge)

Inspired by testing of bi-directional communication between ROS and ROS2 nodes can be seen in the [Repository](https://github.com/ros2/ros1_bridge) of ros1bridge

### Example 1a: ROS 2 talker and ROS 1 listener

First we start a ROS 1 `roscore`:

```
# Shell A (ROS 1 only):
. /opt/ros/melodic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
roscore
```

---

Then we start the dynamic bridge which will watch the available ROS 1 and ROS 2 topics.
Once a *matching* topic has been detected it starts to bridge the messages on this topic.

```
# Shell B (ROS 1 + ROS 2):
# Source ROS 1 first:
. /opt/ros/melodic/setup.bash
# Or, on OSX, something like:
# . ~/ros_catkin_ws/install_isolated/setup.bash
# Source ROS 2 next:
. <install-space-with-bridge>/setup.bash
# For example:
# . /opt/ros/eloquent/setup.bash
export ROS_MASTER_URI=http://localhost:11311
ros2 run ros1_bridge dynamic_bridge
```

The program will start outputting the currently available topics in ROS 1 and ROS 2 in a regular interval.

---

Now we start the ROS 2 talker from the papyrus workspace.

```
# Shell C:
# Source ROS 2 next:
. <install-space-with-ros2>/setup.bash
ros2 run knowrobwrapper KnowRobWrapper
```

The ROS 2 node will start printing the published messages to the console.

---

From the catkin workspace, perform

```
# Shell C:
# Source ROS 1 first:
. ~/ros_catkin_ws/devel/setup.bash
roslaunch json_prolog json_prolog.launch
```
This will launch the interface to the prolog.

---

Now we start the ROS 1 listener. 
From the catkin workspace, run the wrapper 

```
# Shell D:
. ~/ros_catkin_ws/devel/setup.bash
rosrun json_prolog json_prolog_wrapper
```

The ROS 1 node will start printing the received messages to the console.

---

When looking at the output in *shell B* there will be a line stating that the bridge for this topic has been created:

```
created 1to2 bridge for topic '/chatter' with ROS 1 type 'std_msgs/String' and ROS 2 type 'std_msgs/String'
```

At the end stop all programs with `Ctrl-C`.
Once you stop either the talker or the listener in *shell B* a line will be stating that the bridge has been torn down:

```
removed 1to2 bridge for topic '/chatter'
```


