from launch import LaunchDescription
import launch.actions
from launch_ros.substitutions import ExecutableInPackage
from launch_ros.actions import LifecycleNode
from ament_index_python.packages import get_package_share_directory

share_dir = get_package_share_directory('publishsubscribe')

def generate_launch_description():

	# Launch Description
	ld = launch.LaunchDescription()

	# Add the actions to the launch description.
	# The order they are added reflects the order in which they will be executed.
	ld.add_entity(LifecycleNode(
		node_name='subscriber',
		package='publishsubscribe', node_executable='Subscriber',
		remappings=[
			('rMap', 'publisher/Map/pMap')
		],
		parameters=[share_dir+'/launch/cfg/param.yaml']
	))
	return ld
