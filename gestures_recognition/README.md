# Papyrus4Robotics Hand Gesture Detection 

The project implements the hand gestures detection for a command giving to a robot.
System requirements: ROS2 under Linux Ubuntu 18.04 (tested), should also work under 16.04.

The recognized gestures for now:
ok, go!             |  stop | rock | peace |  attention
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------: |:-------------------------:
<img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qokgo.png" height="220"> | <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qstop2.png" height="220">  |  <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qrock.png" height="220"> | <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qpeace.png" height="220"> | <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qattention.png" height="220">

scroll           |  phone | left | right
:-------------------------:|:-------------------------:|:-------------------------:|:-------------------------:
<img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qscroll.png" height="220"> | <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qphone.png" height="220">  |  <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qleft.png" height="220"> | <img src="https://github.com/EkaterinaKapanzha/images/blob/master/images/qright.png" height="220">


![Image of ok, go!](https://github.com/EkaterinaKapanzha/images/blob/master/images/okgo.PNG)

## Dependencies.

1. Make sure you have ros2 and colcon, cmake (``sudo apt-get install cmake``), python (``sudo apt-get install python``), python3 (``sudo apt-get install python3``), pip (``sudo apt-get install python-pip``), pip3 (``sudo apt-get install python3-pip``)  and numpy (``pip install numpy``) installed

Install the Python “six” library by running ``pip3 install --user six``. 

2. Install bazel, following instructions from https://docs.bazel.build/versions/master/install-ubuntu.html or run bazel-install.sh (sudo rights are necessary, please read bazel-install.sh)

``./bazel-install.sh ``

3. Install OpenCV using apt-get:

``sudo apt update && sudo apt install build-essential git ``

``sudo apt install cmake ffmpeg libavformat-dev libdc1394-22-dev libgtk2.0-dev libjpeg-dev libpng-dev libswscale-dev libtbb2 libtbb-dev libtiff-dev``

``sudo apt-get install libopencv-core-dev libopencv-highgui-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-imgproc-dev libopencv-video-dev ``

## Building project

4. Build mediapipe library

Set Python3 as the default Python version: open your .bashrc file ``nano ~/.bashrc``. Type ``alias python=python3`` on to a new line at the top of the file then save the file with ctrl+o and close the file with ctrl+x. Then, back at your command line type ``source ~/.bashrc``.

Build mediapipe from the <gestures_recognition_root_folder>/mediapipe folder:

`` cd mediapipe/``

``export GLOG_logtostderr=1``

``bazel build -c opt mediapipe/examples/desktop/multi_hand_tracking:multi_hand_tracking_cpu --define MEDIAPIPE_DISABLE_GPU=1``

5. Build ROS2/papyrus components

Don't forget to source your ROS:

``. /opt/ros/setup.bash``

Build ROS2 nodes from the <gestures_recognition_root_folder>/src/papyrus folder:

``cd ../src/papyrus/``

``colcon build --symlink-install``

## Running

6. Go back to the project root folder and run alltogether. In run.sh you can change between single or multi-hand tracking and input from the camera or video files by uncommenting the respective command. If you chose to test with the provided videos, please don't forget to change the path of the video. 

`` cd <gestures_recognition_root_folder>``

``./run.sh ``

7. Run RVIZ in a new terminal window, use config file gesturesRecognition.rviz

``rviz2``

The running setup should look like this:
![Image of running](https://github.com/EkaterinaKapanzha/images/blob/master/images/scenario1.png)


