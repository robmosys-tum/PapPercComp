"""
Default launch script. Launches all three essential nodes.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """
    Use the parameters to determine network files at runtime.
    """
    config = os.path.join(get_package_share_directory('fruit_detection'),
                          'config', 'params.yaml')
    model = os.path.join(get_package_share_directory('fruit_detection'),
                         'networks', 'squeezenet.h5')
    return LaunchDescription([
        Node(package='fruit_detection',
             node_executable='detection_service.py',
             node_name='detection_service',
             parameters=[config]),
        Node(package='fruit_detection',
             node_executable='disease_service.py',
             node_name='disease_service',
             parameters=[{
                 "model_file": model
             }]),
        Node(package='fruit_detection',
             node_executable='FruitDetection',
             node_name='FruitDetection')
    ])
