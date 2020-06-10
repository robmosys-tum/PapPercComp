#ifndef KINECTFUSION_CAMERA_H
#define KINECTFUSION_CAMERA_H

/*
 * Camera class declarations. Add your own camera handler by deriving from DepthCamera.
 * Author: Christian Diller
 * Adapted by: Daniel Hettegger in 2020 
 */

#include <data_types.h>


using kinectfusion::CameraParameters;

/**
 * Represents a single input frame
 * Packages a depth map with the corresponding RGB color map
 * The depth map is expected to hold float values, the color map 8 bit RGB values
 */
struct InputFrame {
    cv::Mat_<float> depth_map;
    cv::Mat_<cv::Vec3b> color_map;
};

/*
 * Models the interface to a device that provides raw depth images
 */
class DepthCamera {
public:
    virtual ~DepthCamera() = default;

    virtual InputFrame grab_frame() const = 0;
    virtual CameraParameters get_parameters() const = 0;
};


/*
 * Provides depth frames acquired by a Microsoft Kinect camera.
 */

class KinectCamera : public DepthCamera {
public:
    KinectCamera();
    ~KinectCamera();
    CameraParameters get_parameters() const override;
private:
    CameraParameters _cam_params;
};
 

#endif //KINECTFUSION_CAMERA_H