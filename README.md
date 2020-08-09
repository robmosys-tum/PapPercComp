# Papyrus4Robotics Perception Components

#build mediapipe library
``bazel build -c opt mediapipe/examples/desktop/hand_tracking:hand_tracking_cpu --define MEDIAPIPE_DISABLE_GPU=1``

#build ROS2/papyrus components
``cd src/papyrus/
colcon build --symlink-install``

#run alltogether
#you can change single or multi-hand tracking and input from the camera or video files
`` cd ../..
./run.sh ``


