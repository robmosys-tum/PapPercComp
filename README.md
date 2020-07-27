# Papyrus4Robotics Perception Components

## Dual-Arm Grasping of Chairs

### Installation

First, we install the a library for Fast Gauss Transforms which is the only non-ROS system wide dependency for this project.

    $ cd ~
    $ git clone https://github.com/gadomski/fgt
    $ cd fgt
    $ mkdir build
    $ cd build
    $ cmake ..
    $ sudo make install

Next, we setup the workspace for the project.

    $ cd ~
    $ mkdir chair_manipulation_ws
    $ cd chair_manipulation_ws
    $ mkdir src
    $ catkin init --workspace .
    $ source devel/setup.bash
    $ git clone --recurse-submodules https://github.com/robmosys-tum/PapPercComp -b chair_manipulation src/chair_manipulation
    $ wstool init src
    $ wstool merge -t src src/chair_manipulation/dependencies.rosinstall
    $ wstool update -t src
    $ rosdep install -y --from-paths src --ignore-src --rosdistro ${ROS_DISTRO}
    $ catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release 
    $ catkin build
