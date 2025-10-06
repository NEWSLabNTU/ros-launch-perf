from typing import List
from typing import Optional
from typing import Dict
from typing import Union


import launch
from launch_ros.actions.node import Node
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.utilities import add_node_name
from launch_ros.utilities import get_node_name_count
from launch.utilities import perform_substitutions, is_a
from launch_ros.descriptions import Parameter
from launch.utilities import normalize_to_list_of_substitutions

from ..utils import param_to_kv

from .execute_process import visit_execute_process
from ..launch_dump import LaunchDump, NodeRecord


def visit_node(
    node: Node, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
    node._perform_substitutions(context)

    def substitute(subst):
        nonlocal context
        return perform_substitutions(context, normalize_to_list_of_substitutions(subst))

    executable = substitute(node.node_executable)
    package = substitute(node.node_package)

    if node._Node__ros_arguments is not None:
        ros_args = list(substitute(subst) for subst in node._Node__ros_arguments)
    else:
        ros_args = None

    if node._Node__arguments is not None:
        args = list(substitute(subst) for subst in node._Node__arguments)
    else:
        args = None

    if node.expanded_node_namespace == node.UNSPECIFIED_NODE_NAMESPACE:
        namespace = None
    else:
        namespace = node.expanded_node_namespace

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

    # Visit ExecuteProcess
    ret = visit_execute_process(node, context, dump)

    if node.is_node_name_fully_specified():
        add_node_name(context, node.node_name)
        node_name_count = get_node_name_count(context, node.node_name)
        if node_name_count > 1:
            execute_process_logger = launch.logging.get_logger(node.name)
            execute_process_logger.warning(
                "there are now at least {} nodes with the name {} created within this "
                "launch context".format(node_name_count, node.node_name)
            )

    # Extract parameters
    params_files = list()
    params = list()
    node_params = node._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                path = entry
                try:
                    with open(path, "r") as fp:
                        data = fp.read()
                        params_files.append(data)
                        dump.file_data[path] = data
                except (FileNotFoundError, IOError) as e:
                    execute_process_logger = launch.logging.get_logger(node.name)
                    execute_process_logger.error(
                        f"Unable to read parameter file {path}: {e}"
                    )
                    # Continue without this params file
            else:
                assert is_a(entry, Parameter)
                name, value = param_to_kv(entry)
                params.append((name, value))

    if node.expanded_remapping_rules is None:
        remaps = list()
    else:
        remaps = node.expanded_remapping_rules

    # Store a node record
    record = NodeRecord(
        executable=executable,
        package=package,
        name=node._Node__expanded_node_name,
        namespace=namespace,
        exec_name=node.name,
        cmd=node.cmd,
        remaps=remaps,
        params=params,
        params_files=params_files,
        ros_args=ros_args,
        args=args,
    )
    dump.node.append(record)

    return ret
