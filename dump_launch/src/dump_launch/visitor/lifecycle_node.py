from typing import List
from typing import Optional

from launch_ros.actions.lifecycle_node import LifecycleNode
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity


from .node import visit_node
from ..launch_dump import LaunchDump


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

    # Record the lifecycle node name
    node_name = node._Node__expanded_node_name
    dump.lifecycle_node.append(node_name)

    return visit_node(node, context, dump)
