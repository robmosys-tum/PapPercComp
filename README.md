# Papyrus4Robotics Hand Gesture Detection 

1. Set Python 3.7 as the default Python version and install the Python “six” library by running ``pip3 install --user six``. 
Install bazel, following instructions from https://docs.bazel.build/versions/master/install-ubuntu.html or run bazel-install.sh (sudo rights are necessary, please read bazel-install.sh)

``./bazel-install.sh ``

2. Make sure you have cmake, python, python3, pip and numpy installed

3. Install OpenCV it using apt-get:

``sudo apt update && sudo apt install build-essential git ``

``sudo apt install cmake ffmpeg libavformat-dev libdc1394-22-dev libgtk2.0-dev libjpeg-dev libpng-dev libswscale-dev libtbb2 libtbb-dev libtiff-dev``

``sudo apt-get install libopencv-core-dev libopencv-highgui-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-imgproc-dev libopencv-video-dev ``

4. Build mediapipe library

``export GLOG_logtostderr=1

bazel build -c opt mediapipe/examples/desktop/hand_tracking:hand_tracking_cpu --define MEDIAPIPE_DISABLE_GPU=1``

5. Build ROS2/papyrus components

``cd src/papyrus/``

``colcon build --symlink-install``

6. Run alltogether, in run.sh you can change between single or multi-hand tracking and input from the camera or video files

`` cd ../..``

``./run.sh ``

7. Run RVIZ, use config file gesturesRecognition.rviz

``rviz2``


