from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import cast
from typing import Dict


from launch.actions import ExecuteProcess
from launch_ros.actions.node import Node
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.utilities import add_node_name
from launch_ros.utilities import get_node_name_count

from ..dump import LaunchDump


def visit_node(
    node: Node, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
    node._perform_substitutions(context)
    # Prepare the ros_specific_arguments list and add it to the context so that the
    # LocalSubstitution placeholders added to the the cmd can be expanded using the contents.
    ros_specific_arguments: Dict[str, Union[str, List[str]]] = {}
    if node._Node__node_name is not None:
        ros_specific_arguments["name"] = "__node:={}".format(
            node._Node__expanded_node_name
        )
    if node._Node__expanded_node_namespace != "":
        ros_specific_arguments["ns"] = "__ns:={}".format(
            node._Node__expanded_node_namespace
        )

    # Give extensions a chance to prepare for execution
    for extension in node._Node__extensions.values():
        cmd_extension, ros_specific_arguments = extension.prepare_for_execute(
            context, ros_specific_arguments, node
        )
        node.cmd.extend(cmd_extension)

    context.extend_locals({"ros_specific_arguments": ros_specific_arguments})
    ret = ExecuteProcess.execute(node, context)

    if node.is_node_name_fully_specified():
        add_node_name(context, node.node_name)
        node_name_count = get_node_name_count(context, node.node_name)
        if node_name_count > 1:
            execute_process_logger = launch.logging.get_logger(node.name)
            execute_process_logger.warning(
                "there are now at least {} nodes with the name {} created within this "
                "launch context".format(node_name_count, node.node_name)
            )

    return ret
