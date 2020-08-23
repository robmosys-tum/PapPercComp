#!/bin/bash
killall HandTrackerComponent
# this script is meant to run the entire solution
# prepare exit if necessary

_term() {
  echo "Exitting"
  kill -TERM "$Gesture" 2>/dev/null
  kill -TERM "$library" 2>/dev/null
  kill -KILL "$Tracker" 2>/dev/null
}

trap _term SIGTERM

# source
source src/papyrus/install/setup.bash

# run handtracker_wrapper

ros2 run handtrackercomponent HandTrackerComponent &
Tracker=$!
#ros2 run handtracker_wrapper handtracker_wrapper &
#wrapper=$!

# run gesture_recognition
ros2 run gesturedetectorcomponent GestureDetectorComponent &
Gesture=$!

ros2 run rvizvisualizationcomponent RVIZVisualizationComponent &
Gesture=$!

sleep 3

# run library
cd mediapipe

#default option: input from camera, multi-hand tracking
bazel-bin/mediapipe/examples/desktop/multi_hand_tracking/multi_hand_tracking_cpu   --calculator_graph_config_file=mediapipe/graphs/hand_tracking/multi_hand_tracking_desktop_live.pbtxt &

#uncomment if you want to use prerecorded video
#bazel-bin/mediapipe/examples/desktop/multi_hand_tracking/multi_hand_tracking_cpu   --calculator_graph_config_file=mediapipe/graphs/hand_tracking/multi_hand_tracking_desktop_live.pbtxt --input_video_path=/home/katya/PapPercComp/gestures_recognition/Video1.mp4 &

#uncomment if you want to use single hand
#bazel-bin/mediapipe/examples/desktop/hand_tracking/hand_tracking_cpu   --calculator_graph_config_file=mediapipe/graphs/hand_tracking/hand_tracking_desktop_live.pbtxt  --input_video_path=/home/katya/PapPercComp/gestures_recognition/Video1.mp4 &

library=$!




wait "$library"
wait "$Gesture"
wait "$Tracker"

#killall HandTrackerComponent
