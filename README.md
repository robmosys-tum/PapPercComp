# Papyrus4Robotics Hand Gesture Detection 

1. Install bazel, following instructions from https://docs.bazel.build/versions/master/install-ubuntu.html or run bazel-install.sh (sudo rights are necessary, please read bazel-install.sh)

``./bazel-install.sh ``

2. Run setup_opencv.sh to automatically build OpenCV from source and modify MediaPipe’s OpenCV config:

``cd mediapipe/``

``./setup_opencv.sh ``

or install it using apt-get:

`` sudo apt-get install libopencv-core-dev libopencv-highgui-dev libopencv-calib3d-dev libopencv-features2d-dev libopencv-imgproc-dev libopencv-video-dev ``

3. Build mediapipe library

``bazel build -c opt mediapipe/examples/desktop/hand_tracking:hand_tracking_cpu --define MEDIAPIPE_DISABLE_GPU=1``

4. Build ROS2/papyrus components

``cd src/papyrus/``

``colcon build --symlink-install``

5. Run alltogether, in run.sh you can change between single or multi-hand tracking and input from the camera or video files

`` cd ../..``

``./run.sh ``


