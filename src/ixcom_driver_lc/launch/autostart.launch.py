from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, LogInfo, RegisterEventHandler
from launch.actions import Shutdown
from launch.events.matchers import matches_action
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import LifecycleNode
from launch_ros.event_handlers.on_state_transition import OnStateTransition
from launch_ros.events.lifecycle.change_state import ChangeState
import lifecycle_msgs.msg


def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    publisher_config_file = LaunchConfiguration('publisher_config_file')
    # adapter_config_file = LaunchConfiguration('adapter_config_file')

    #ixcom_driver_lc_node = LifecycleNode(
    #    package='ixcom_driver_lc',
    #    executable='ixcom_driver_lifecycle_node',
    #    namespace=namespace,
    #    name='ixcom_driver_lifecycle_node',
    #    output='screen',
    #    parameters=[
    #        publisher_config_file,
    #        adapter_config_file
    #    ],
    #    arguments=['--ros-args', '--log-level', 'INFO'],
    #    on_exit=Shutdown()
    #)

    ixcom_driver_lc_node = LifecycleNode(
	package="ixcom_driver_lc",
	executable="ixcom_driver_lifecycle_node",
	namespace=namespace,
	name="ixcom_driver_lifecycle_node",
	output="screen",
	parameters=[
		publisher_config_file
	],
	arguments=['--ros-args', '--log-level', 'INFO'],
	on_exit=Shutdown()
    )

    # Changes the lifecycle state of the ixcom_driver_lc_node to configure
    ixcom_driver_lc_configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ixcom_driver_lc_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE))

    # Changes the lifecycle state of the ixcom_driver_lc_node to active
    ixcom_driver_lc_activate_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(ixcom_driver_lc_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE))

    # Event handler that gets called, when the ixcom_driver_lc_node gets
    # transitioned into the inactive state
    ixcom_driver_lc_inactive_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ixcom_driver_lc_node,
            goal_state='inactive',
            entities=[
                # LogInfo(msg = 'ixcom_driver_lc_node reached state "inactive"'),
                ixcom_driver_lc_activate_event
            ]
        )
    )

    # Event handler that gets called, when the ixcom_driver_lc_node gets
    # transitioned into the active state
    ixcom_driver_lc_active_state_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=ixcom_driver_lc_node,
            goal_state='active',
            entities=[
                LogInfo(msg='ixcom_driver_lc_node reachted state "active"'),
            ]
        )
    )

    # The automated lifecycle transition flow look as follows:
    # 1. The node is in the unconfigured state, after startup
    # 2. The ixcom_driver_lc_configure_event is triggered automatically, because
    #    it's part of the LaunchDescription.
    #    This kicks of the automated state transitioning process of the ixcom_driver_lc_node
    # 3. After the node transitions into the configured state, the event handler
    #    "ixcom_driver_configure_event" is triggered,
    #    which is part of this launch description. It internally logs the current
    #    state and emits the
    #    "ixcom_driver_lc_activate_event", because its part of the event handlers entity
    #    list (which contains the actions that
    #    are executed, after the event took place). This start the state transition
    #    into the active state
    # 4. After the node has transitioned into the active state, the
    # ixcom_driver_lc_active_state_handler is called,
    #    which logs the successful transition to the active state to the console

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace'),
        DeclareLaunchArgument('publisher_config_file',
                              default_value=get_package_share_directory('ixcom_driver_lc')
                              + '/params/publisher_config.yml',
                              description='Path to a parameter file that will be passed '
                                          'to the ixcom_driver_lc_node'),
        #DeclareLaunchArgument('adapter_config_file',
        #                      default_value=get_package_share_directory('ixcom_driver_lc')
        #                      + '/params/adapter_config.yml',
        #                      description='Path to the service adapter parameter file that will'
        #                                  'be passed to the ixcom_driver_lc_node'),
        # Add the ixcom_driver_lc_node, as well as the necessary event handlers for automated
        # lifecycle state transitioning
        ixcom_driver_lc_node,
        ixcom_driver_lc_configure_event,
        ixcom_driver_lc_inactive_state_handler,
        ixcom_driver_lc_active_state_handler
    ])
