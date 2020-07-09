# Fruit and Disease Detection as ROS2 component
**Author** [Philipp Seitz](https://github.com/Gistbatch)

This package contains a component created with Papyrus for robotics.
![component](https://github.com/robmosys-tum/PapPercComp/blob/fruit_detection/images/fruit_detection.compdef.png) 

## Overview

The goal of this project was to provide object detection for fruits and identfiy diseases in them.
It uses a two step approach:
* Detect fruit objects in an image
* Classifiy diseases for the detected fruits

The steps are realized with neural networks implemented in TensorFlow.
For detection a faster_rcnn is used. For classification a SqueezeNet is used.

## ROS2
This package was created for ROS2-Eloquent. It contains a launch file [fruit_detection_launch](https://github.com/robmosys-tum/PapPercComp/blob/fruit_detection/launch/fruit_detection_launch.py).
The node interaction is described here ![interaction](https://github.com/robmosys-tum/PapPercComp/blob/fruit_detection/images/overview.png).

### Nodes
The following nodes are declared:
* FruitDetection is the main component. The node processes images in the
/image topic and publishes the results to /BoxImage
* detection_service.py (service) is a service wich detects fruits using a 
faster_rcnn. *Parameters: module_handle* URL for the model.
* disease_service.py (service) is a service wich classifies fruit diseases using a 
neural network. *Parameters: model_file* filepath for the classification network.

### Messages
The package declares two new message types:
* ClassImage
```c
string fruit
int32 fruit_score
float32 xmin
float32 xmax
float32 ymin
float32 ymax
string disease "undefined"
int32 disease_score 0
```
* BoxesImage
```c
sensor_msgs/Image img
ClassBox[] boxes
```

## Install
This package is not published oficially and has to be built from source.
To do this clone the package and build it as an regular ROS2 package using *colcon*.
Since this packages contains python and c++ nodes some additional steps are required.
ROS2 supports *python3*. Make sure you have an correct executable at *usr/bin/python3*
and linked correctly `alias python="python3"`
Additionally virtual environments are not supported which requires you to make sure *pip3*
is set e.g. `alias pip="pip3"`.
Additionally some dependencies need to be installed manually:
```bash
python3 -m pip install pip --upgrade
pip3 install tensorflow tensorflow-hub numpy opencv-python
```
The neural networks are built with TensorFlow2. It is possible to use a dedicated
GPU to speed up the prediction process in both services. The necessary steps are
documented [here](https://www.tensorflow.org/install/gpu).

## Usage

The package is straight forward to use. Any image published will be picked up and
processed. The nodes can also be launched individually. You can customize the neural
networks by doing the following:
* Object Detection: To change the object detection you can use the *module_handle* parameter.
* Disease Classification: To change the classification network you can use the *model_file* parameter.

Simple example:

1. Change to your ros2 workspace: `cd <ros2_ws>`
2. Build the project: `colcon build --packages-select fruit_detection`
3. Source the setup: `source install/setup.bash`
4. Launch the nodes: `ros2 launch fruit_detection fruit_detection_launch.py`
5. Unzip the example: `unzip src/fruit_detection/example/example.zip -d src/fruit_detection/example`
6. Play the example bag: `ros2 bag play src/fruit_detection/example/bag.db3`

Keep in mind that logging doesn't work properly when using `ros2 launch`.
If you use the the nodes individually make sure to set their parameters!
Sometimes playing the bag doesn't get registered in the *FruitDetection* node.
In this case just replay the bag.

## Develop

The development of the FruitDetection node was done with Papyrus. It is
recommended to use it for further development.
The scripts are writen in plain python3. If you add additional dependencies make
sure to add them to the installation part of this readme since ROS2 does not 
specify a way to add them to the package directly. If you add additional folders
make sure you add them in the CMakeLists.txt.
The code was linted with pylint and cpplint respectively. 