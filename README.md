# RGBD Surface Reconstruction

## Introduction

This repository contains code for surface reconstrucion using ros 2. To improve surface reconstruction quality, the tsdf fusion weighting has been adapted to include quality estimation based on the Kinect V2 Sensor noise model.

## Dependencies

* NVIDA GPU with CUDA support
* CUDA Libraries and Headers (tested with 10.2)
* OpenCV (tested with OpenCV 3.4.10)


## Usage

* clone this repository into your ros 2 workspace using the following command:

    git clone --single-branch --branch rgbd_surface_reconstruction https://github.com/robmosys-tum/PapPercComp --recurse-submodules 


* Build the packages using colcon.
* Download scans from the corbs dataset: [Here](http://corbs.dfki.uni-kl.de/) \
  For every scan download the pre processed data as well as the trajectory.
* Update the path to your dataset and trajectory in the launchfile /rgbd_surface_reconstruction/launch/launch_reconstruciton_demo.py 
* Launch the nodes using:\
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_demo.py 
