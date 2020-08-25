import launch.actions
import launch_ros
import lifecycle_msgs.msg

from launch_ros.events.lifecycle import ChangeState
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch_ros.events.lifecycle import ChangeState
from ament_index_python.packages import get_package_share_directory

share_dir = get_package_share_directory('handgesturedetectionsystem')

def generate_launch_description():

	# Launch Description
	ld = launch.LaunchDescription()

	# Add the actions to the launch description.
	# The order they are added reflects the order in which they will be executed.
	ComponentInstance1_node = LifecycleNode(
		node_name='ComponentInstance1',
		package='cameradrivercomponent', node_executable='CameraDriverComponent',
		remappings=[
			('video_output', 'ComponentInstance1/Image/video_output')
		],
		parameters=[share_dir+'/launch/cfg/param.yaml'],
		output='screen',
		emulate_tty=True	# assure that RCLCPP output gets flushed
	)
	ld.add_entity(ComponentInstance1_node)
	ComponentInstance2_node = LifecycleNode(
		node_name='ComponentInstance2',
		package='handtrackercomponent', node_executable='HandTrackerComponent',
		remappings=[
			('video_input', 'ComponentInstance1/Image/video_output'), ('points_output', 'ComponentInstance2/PoseArray/points_output')
		],
		parameters=[share_dir+'/launch/cfg/param.yaml'],
		output='screen',
		emulate_tty=True	# assure that RCLCPP output gets flushed
	)
	ld.add_entity(ComponentInstance2_node)
	ComponentInstance4_node = LifecycleNode(
		node_name='ComponentInstance4',
		package='gesturedetectorcomponent', node_executable='GestureDetectorComponent',
		remappings=[
			('points_input', 'ComponentInstance2/PoseArray/points_output')
		],
		parameters=[share_dir+'/launch/cfg/param.yaml'],
		output='screen',
		emulate_tty=True	# assure that RCLCPP output gets flushed
	)
	ld.add_entity(ComponentInstance4_node)

	return ld
