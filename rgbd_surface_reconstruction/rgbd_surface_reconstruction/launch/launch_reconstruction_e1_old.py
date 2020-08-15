from launch import LaunchDescription
from launch_ros.actions import Node

import os
from zipfile import ZipFile
import urllib
import signal

def signal_handler(sig, frame):
    quit()

original_sigint_handler = signal.getsignal(signal.SIGINT)
signal.signal(signal.SIGINT, signal_handler)

def download_dataset(corbs_datasetpath, dataset_url):
    corbs_zip_path = corbs_datasetpath+'/zipfolder.zip'
    urllib.request.urlretrieve(dataset_url, corbs_zip_path)
    with ZipFile(corbs_zip_path, 'r') as zip_ref:
        zip_ref.extractall(corbs_datasetpath)
    os.remove(corbs_zip_path)

def generate_launch_description():
    corbs_datasetpath = os.path.abspath(os.path.join(os.path.dirname( __file__ ), '../', 'datasets'))
    
    download_indicator = corbs_datasetpath + '/E1_T'
    dataset_url = "http://corbs.dfki.uni-kl.de/suppfiles/ElectricalCabinet/E1/E1_Trajectory.zip"
    if not os.path.isfile(download_indicator):
        download_dataset(corbs_datasetpath, dataset_url)
        open(download_indicator, 'a').close()

    download_indicator = corbs_datasetpath + '/E1_D'
    dataset_url = "http://corbs.dfki.uni-kl.de/suppfiles/ElectricalCabinet/E1/E1_pre_registereddata.zip"
    if not os.path.isfile(download_indicator):
        print("Dataset not found!")
        print("Downloading dataset please wait... (This will take a few minutes)")
        download_dataset(corbs_datasetpath, dataset_url)
        open(download_indicator, 'a').close()

    signal.signal(signal.SIGINT, original_sigint_handler)
    return LaunchDescription([
        Node(
            package='ros2_corbs_publisher',
            node_executable='ros2_corbs_publisher_node',
            parameters=[
                {"data_path": corbs_datasetpath+"/E1_pre_registereddata/",
                 "trajectory_path": corbs_datasetpath+"/E1_Trajectory/"}],
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
                 "offset_x": 100.0,
                 "offset_y": +100.0,
                 "offset_z": -350.0,
                 "use_kinect_noise_model": False,
                 "use_every_nth_frame": 1,
                 "export_frame" : 600,
                 "export_name": "model_e1_w500"
                 }],
            output='screen',
            emulate_tty=True,
        )
    ])

