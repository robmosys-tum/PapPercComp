#include "Rgbd_surface_reconstructionCompdef/depth_camera.h"

//### Kinect ###
KinectCamera::KinectCamera()
{
    _cam_params.image_width = 640;
    _cam_params.image_height = 480;
    _cam_params.focal_x = 468.60;
    _cam_params.focal_y = 468.61;
    _cam_params.principal_x = _cam_params.image_width / 2 - 0.5f;
    _cam_params.principal_y = _cam_params.image_height / 2 - 0.5f;
}
KinectCamera::~KinectCamera()
{
}
CameraParameters KinectCamera::get_parameters() const
{
    return _cam_params;
}
