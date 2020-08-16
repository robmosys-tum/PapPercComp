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

	# transition to configure after startup
	configure_ComponentInstance1 = launch.actions.RegisterEventHandler(
		launch.event_handlers.on_process_start.OnProcessStart(
			target_action=ComponentInstance1_node,
			on_start=[
	 			launch.actions.EmitEvent(
					event=launch_ros.events.lifecycle.ChangeState(
						lifecycle_node_matcher=launch.events.matches_action(ComponentInstance1_node),
						transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
					)
				)
			]
		)
	)
	ld.add_entity(configure_ComponentInstance1)
	configure_ComponentInstance2 = launch.actions.RegisterEventHandler(
		launch.event_handlers.on_process_start.OnProcessStart(
			target_action=ComponentInstance2_node,
			on_start=[
	 			launch.actions.EmitEvent(
					event=launch_ros.events.lifecycle.ChangeState(
						lifecycle_node_matcher=launch.events.matches_action(ComponentInstance2_node),
						transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
					)
				)
			]
		)
	)
	ld.add_entity(configure_ComponentInstance2)
	configure_ComponentInstance4 = launch.actions.RegisterEventHandler(
		launch.event_handlers.on_process_start.OnProcessStart(
			target_action=ComponentInstance4_node,
			on_start=[
	 			launch.actions.EmitEvent(
					event=launch_ros.events.lifecycle.ChangeState(
						lifecycle_node_matcher=launch.events.matches_action(ComponentInstance4_node),
						transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE
					)
				)
			]
		)
	)
	ld.add_entity(configure_ComponentInstance4)

	# transition to activate, once inactive
	activate_ComponentInstance1 = launch.actions.RegisterEventHandler(
		launch_ros.event_handlers.OnStateTransition(
			target_lifecycle_node=ComponentInstance1_node,
			start_state='configuring', goal_state='inactive',
			entities=[
				launch.actions.EmitEvent(
					event=launch_ros.events.lifecycle.ChangeState(
						lifecycle_node_matcher=launch.events.matches_action(ComponentInstance1_node),
						transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
					)
				)
			]
		)
	)
	ld.add_entity(activate_ComponentInstance1)
	activate_ComponentInstance2 = launch.actions.RegisterEventHandler(
		launch_ros.event_handlers.OnStateTransition(
			target_lifecycle_node=ComponentInstance2_node,
			start_state='configuring', goal_state='inactive',
			entities=[
				launch.actions.EmitEvent(
					event=launch_ros.events.lifecycle.ChangeState(
						lifecycle_node_matcher=launch.events.matches_action(ComponentInstance2_node),
						transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
					)
				)
			]
		)
	)
	ld.add_entity(activate_ComponentInstance2)
	activate_ComponentInstance4 = launch.actions.RegisterEventHandler(
		launch_ros.event_handlers.OnStateTransition(
			target_lifecycle_node=ComponentInstance4_node,
			start_state='configuring', goal_state='inactive',
			entities=[
				launch.actions.EmitEvent(
					event=launch_ros.events.lifecycle.ChangeState(
						lifecycle_node_matcher=launch.events.matches_action(ComponentInstance4_node),
						transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE
					)
				)
			]
		)
	)
	ld.add_entity(activate_ComponentInstance4)

	return ld
