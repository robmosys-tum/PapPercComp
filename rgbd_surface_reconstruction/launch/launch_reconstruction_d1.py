

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_corbs_publisher',
            node_executable='ros2_corbs_publisher_node',
            parameters=[
                {"data_path": "/home/drkmtr/Downloads/CORBS/D1_pre_registereddata/",
                 "trajectory_path": "/home/drkmtr/Downloads/CORBS/D1_Trajectory/"}],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rgbd_surface_reconstruction',
            node_executable='Rgbd_surface_reconstruction',
            parameters=[{
                 "voxel_scale": 2.5,
                 "volume_size_x": 512,
                 "volume_size_y": 800,
                 "volume_size_z": 512,
                 "offset_x": 200.0,
                 "offset_y": +200.0,
                 "offset_z": -500.0,
                 "use_kinect_noise_model": False,
                 "use_every_nth_frame": 1,
                 "export_frame" : 500,
                 "export_name": "model_d1_w500"
                 }],
            output='screen',
            emulate_tty=True,
        )
    ])

