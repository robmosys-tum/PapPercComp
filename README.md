# Papyrus4Robotics Perception Components

## Dual-Arm Grasping of Chairs

### Installation

First, we have to install two libraries system wide from source: fgt and fcl.

    $ cd ~
    $ git clone https://github.com/gadomski/fgt
    $ cd fgt
    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install
    $ cd ~
    $ git clone https://github.com/flexible-collision-library/fcl
    $ cd fcl
    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install

Now, we install a compiler cache to decrease the compile time.

    $ sudo apt-get install ccache
    $ echo 'export PATH=/usr/lib/ccache:$PATH' >> $HOME/.bashrc
    $ source $HOME/.bashrc 

Next, we setup the workspace for the project.

    $ cd ~
    $ mkdir chair_manipulation_ws
    $ cd chair_manipulation_ws
    $ mkdir src
    $ catkin init --workspace .
    $ git clone --recurse-submodules https://github.com/robmosys-tum/PapPercComp -b chair_manipulation src/chair_manipulation
    $ wstool init src
    $ wstool merge -t src src/chair_manipulation/dependencies.rosinstall
    $ wstool update -t src
    $ rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
    $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release 
    $ catkin build

By running the build in an Ubuntu 18 virtual machine, I sometimes get the compiler error message

    c++: internal compiler error: Killed (program cc1plus)

In that case, just run catkin build again.

### Usage

First, we have to create the grasp database

    $ source devel/setup.bash
    $ roslaunch chair_manipulation_grasp_detection_advanced create_grasp_database.launch

To run the simulation, we need four terminals.

Bringing up the scene

    $ source devel/setup.bash
    $ roslaunch chair_manipulation_gazebo scene_bringup_advanced.launch

Now, you have to switch to gazebo and press the play button.
It seems to be more stable to start gazebo in the paused mode.

Bringup moveit

    $ source devel/setup.bash
    $ roslaunch chair_manipulation_gazebo moveit_bringup.launch

Start the node that lifts the chair

    $ source devel/setup.bash
    $ roslaunch chair_manipulation_gazebo lift.launch

Start the detection

    $ source devel/setup.bash
    $ roslaunch chair_manipulation_gazebo detection_advanced.launch debug:=true

Note that it may sometimes happen that some nodes do not launch correctly.
So just Ctrl+C and run again if something does not work at the first time.