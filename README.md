# RGBD Surface Reconstruction

## Introduction

This repository contains code for surface reconstrucion using ros 2. To improve surface reconstruction quality, the tsdf fusion weighting has been adapted to include quality estimation based on the Kinect V2 Sensor noise model.

## Dependencies

* NVIDA GPU with CUDA support (and probably a good amount of VRAM)
* CUDA Libraries and Headers (tested with 10.2)
* OpenCV (tested with OpenCV 3.4.10)
* Eigen 3

## Installation 

* install CUDA by following [this guide](https://docs.nvidia.com/cuda/cuda-installation-guide-linux/index.html).
* install Eigen 3. [See here](https://eigen.tuxfamily.org/dox/GettingStarted.html).
* install OpenCV 3.4 or above use for example [this guide](https://gist.github.com/raulqf/f42c718a658cddc16f9df07ecc627be7) for Ubuntu 18.04 and OpenCV 4.2

## Usage

* clone this repository into your ros2 workspace using the following command:

    git clone --single-branch --branch rgbd_surface_reconstruction https://github.com/robmosys-tum/PapPercComp --recurse-submodules 


* Build the packages using colcon.
* Launch the nodes using:\
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_h1_update.py \
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_h1_old.py \
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_e1_update.py \
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_e1_old.py \
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_d1_update.py \
    ros2 launch rgbd_surface_reconstruction launch_reconstruciton_d1_old.py
