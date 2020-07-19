

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_corbs_publisher',
            node_executable='ros2_corbs_publisher_node',
            parameters=[
                {"data_path": "/home/drkmtr/Downloads/CORBS/E1_pre_registereddata/",
                 "trajectory_path": "/home/drkmtr/Downloads/CORBS/E1_Trajectory/"}],
            output='screen',
            emulate_tty=True,
        ),
        Node(
            package='rgbd_surface_reconstruction',
            node_executable='Rgbd_surface_reconstruction',
            output='screen',
            emulate_tty=True,
        )
    ])

