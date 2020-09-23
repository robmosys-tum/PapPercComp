# Kitchen Utensil Classifier

This Project presents a ROS2 node capable of classifying 7 different kitchen utensils, though
this is dynamically expandable by exchanging the model and correctly configuring the data/class\_list.txt
file (see section 1).


## 0. Requirements

While OpenCV version >= 3.4 is required as we make use of the dnn module, we recommend using
version >= 4.2, as that is the version we tested with primarily. The ROS2 version used during
testing was Eloquent Elusor, so we recommend using this version or newer as well.

Additionally, we optionally require various python packages for training the network using our
Python notebook provided in the models subfolder. For more details, see section 5.


## 1. Classifier Setup

For using the most "up-to-date" model for classification, we recommend using our script to
download the current model from Google Drive:

    ./scripts/download_model.sh

If there are any problems with this, or if you want to use your own model, there is also the
possibility to simply train your own classifier using a sample Python notebook we provide.
For more details, see section 5.

Via the data/class\_list.txt file, it is possible to configure different outputs: the string at
line i in the file corresponds to the class name of the ith output of the model. By additionally
changing the model stored in data/kitchen\_classifier.onnx, it is possible to expand the currently
available list of classied kitchen utensils, or in fact classify completely different objects.


## 2. Node Setup

Assuming a sourced ROS2 environment, before being able to run the classifier, simply run
the following line in a terminal:

    colcon build && . install/setup.bash


## 3. Running The Classifier

It is worth noting that, due to the usage of relative paths in the code, the node will
only exexcute correctly when run from the directory containing this README file. To run
the node, simply execute the following in the terminal:

    ros2 run kitchenutensilclassifier KitchenUtensilClassifier

At regular intervals, the node should then publish a string describing the class that is
shown in the currently viewed image.


## 4. Testing the node

To see whether the node is working, we suggest using the ros2 image-utils:

    ros2 run image_tools cam2image --ros-args --param-height:=224  --param-width:=224

for starting a camera feed from the system's camera / webcam, and

    ros2 run image_tools showimage

for visualising said feed.


## 5. Classifier Training

The classifier available via the download script we provide was trained using the CutleryClassification.ipynb
notebook file in the models subdirectory. For the sake of dependency simplicity and due to the free
availability, we made use of Google Colab for training purposes, and recommend beginners / those without readily
available hardware to do the same.

The Python version used at the time of writing is 3.6, with the following packages being used in the given versions:

* numpy 1.18
* matplotlib 3.2
* pytorch 1.6
* torchvision 0.7

Simply running the whole notebook is sufficient to generate a model, with the option of downloading it being
available. In the final step, the model is exported to the ONNX (open neural network exchange, see e.g. https://onnx.ai/)
format, thus making it importable and usable by OpenCV's dnn module. Any trained model should be moved to data/kitchen\_classifier.onnx
for it to be loaded by the node.
Additionally, to allow correct outputs, the class\_list.txt file may need changing if different classes were used than those found
in the current file (e.g. by adjusting the script / dataset). See section 1 for more details on the class\_list.txt format.
