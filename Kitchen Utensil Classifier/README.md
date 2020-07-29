# Kitchen Utensil Classifier

## 0. Requirements

While OpenCV version >= 3.4 is required as we make use of the dnn module, we recommend using
version >= 4.2, as that is the version we tested with primarily.


## 1. Classifier Setup

For using the most "up-to-date" model for classification, we recommend using our script to
download the current model from Google Drive:

    ./scripts/download_model.sh

If there are any problems with this, please contact ethan.tatlow@gmail.com

## 2. Node Setup

Assuming a sourced ROS2 environment, before being able to run the classifier, simply run
the following line in a terminal:

    colcon build && . install/setup.bash


## 3. Running The Classifier

It is worth noting that, due to the usage of relative paths in the code, the node will
only exexcute correctly when run from the directory containing this README file. To run
the node, simply execute the following in the terminal:

    ros2 run kitchenutensilclassifier KitchenUtensilClassifier

## 4. Testing the node

To see whether the node is working, we suggest using the ros2 image-utils:

    ros2 run image_tools cam2image --ros-args --param-height:=224  --param-width:=224

for starting a camera feed from the system's camera / webcam, and

    ros2 run image_tools showimage

for visualising said feed.
