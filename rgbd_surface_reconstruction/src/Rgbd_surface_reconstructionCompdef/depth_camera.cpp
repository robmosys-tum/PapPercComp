#include "Rgbd_surface_reconstructionCompdef/depth_camera.h"

//### Kinect ###
KinectCamera::KinectCamera()
{
    _cam_params.image_width = 1920;
    _cam_params.image_height = 1080;
    _cam_params.focal_x = 0.5;
    _cam_params.focal_y = 1;
    _cam_params.principal_x = 1;
    _cam_params.principal_y = 1;
}
KinectCamera::~KinectCamera()
{
}
CameraParameters KinectCamera::get_parameters() const
{
    return _cam_params;
}
