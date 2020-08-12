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
    # we cant define a relative path in the yaml file so we do it here
    model = os.path.join(get_package_share_directory('fruit_detection'),
                         'networks', 'squeezenet_v1.h5')
    return LaunchDescription([
        Node(package='fruit_detection',
             node_executable='detection_service.py',
             node_name='detection_service',
             output='screen',
             emulate_tty=True,
             parameters=[config]),
        Node(package='fruit_detection',
             node_executable='disease_service.py',
             node_name='disease_service',
             output='screen',
             emulate_tty=True,
             parameters=[{
                 "model_file": model
             }, config]),
        Node(package='fruit_detection',
             node_executable='FruitDetection',
             node_name='FruitDetection',
             output='screen',
             emulate_tty=True)
    ])
