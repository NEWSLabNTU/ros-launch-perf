from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
import functools

import launch
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.ros_adapters import get_ros_node
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import StateTransition

import lifecycle_msgs.msg
import lifecycle_msgs.srv


from .node import visit_node
from ..dump import LaunchDump


def visit_lifecycle_node(
    node: LifecycleNode, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
    """
    Execute the action.

    Delegated to :meth:`launch.actions.ExecuteProcess.execute`.
    """
    node._perform_substitutions(context)  # ensure node.node_name is expanded
    if "<node_name_unspecified>" in node.node_name:
        raise RuntimeError("node_name unexpectedly incomplete for lifecycle node")
    node = get_ros_node(context)
    # Create a subscription to monitor the state changes of the subprocess.
    node.__rclpy_subscription = node.create_subscription(
        lifecycle_msgs.msg.TransitionEvent,
        "{}/transition_event".format(node.node_name),
        functools.partial(node._on_transition_event, context),
        10,
    )
    # Create a service client to change state on demand.
    node.__rclpy_change_state_client = node.create_client(
        lifecycle_msgs.srv.ChangeState, "{}/change_state".format(node.node_name)
    )
    # Register an event handler to change states on a ChangeState lifecycle event.
    context.register_event_handler(
        launch.EventHandler(
            matcher=lambda event: isinstance(event, ChangeState),
            entities=[
                launch.actions.OpaqueFunction(function=node._on_change_state_event)
            ],
        )
    )
    # Delegate execution to Node and ExecuteProcess.
    return visit_node(node, context, dump)
