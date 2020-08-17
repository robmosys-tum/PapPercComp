# Papyrus4Robotics Perception Components

# Clone
This project has the TrashNet repository as a submodule so it needs to be cloned with --recurse-submodules, like so:
```
git clone https://github.com/robmosys-tum/PapPercComp --recurse-submodules
```

# Initial Setup
To extract the TrashNet images, run the setup_trashnet.sh script.

To download pretrained ResNet18 and ResNet152 networks, download the zipped models [here](https://drive.google.com/file/d/1rUc1aqVapSxJW06z-MFwzPxBH2rXuxnw/view?usp=sharing).

Then extract it to /PapPercComp/garbage_classification/

# Training a network
A new network can be trained using the Garbage Classifier Training.ipynb notebook. The instructions are written in the notebook.

A trained PyTorch model can be converted into a ScriptModule for use in C++ using the Garbage Classifier Conversion.ipynb notebook. The instructions are also written in the notebook.

# Standalone C++ Inference
A non-ROS2 standalone C++ program is provided to perform inference on a camera stream. To build it, navigate to the cpp/ folder and execute the following commands in the terminal

```
mkdir build
cd build
cmake ..
make
```

The executable can then be used like so: ./garbage-collection `Model Path` `Camera Number`
where `Model Path` is the path to the ScriptModule (.pt) weight file and `Camera Number` is the number of the camera to use. The list of camera numbers can be found by executing `ls /dev/ | grep video`.

# ROS2 Environment
To use the ROS2 components, a working installation of ROS2 is required. For the rest of this section, it is assumed that a valid ROS2 enviornment has been sourced.

Change `PapPercComp/garbage_classification/ros2_ws/garbage_classification/src/Garbage_classificationCompdef/Garbage_classification_impl.cpp` lines 51 and 52 to point to the location of a valid ScriptModule (.pt) weight file and set use_cuda_ accordingly (1 if the weights are CUDA weights and 0 if the weights are CPU weights). Additionally, if a visualization of the input image is desired, uncomment lines 96-99.

Once the desired customizations are completed, build the ROS2 node by exectuing the command `colcon build --symlink-install` inside `PapPercComp/garbage_classification/ros2_ws/`.

Before exectuing the node, the ROS2 enviornment needs to be updated by sourcing the project. This can be done by executing `source PapPercComp/ros2_ws/install/setup.bash`. The node can then be ran with `ros2 run garbage_classification Garbage_classification`.

Once the node is running, input images can be published to the 'image' topic. Camera images can be used by executing `ros2 run image_tools cam2image --ros-args --param width:=640 --param height:=480`. Prerecorded rosbags are also provided [here](), which can be played by executing the command `ros2 bag play [path to directory of bag]`.

