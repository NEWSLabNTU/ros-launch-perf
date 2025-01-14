from dataclasses import dataclass
from typing import List, Optional, Text, Tuple

from launch_ros.actions.node import Node
from launch_ros.descriptions import Parameter
from launch.utilities import is_a

from .substitution import record_substitution
from ..utils import param_to_kv


class NodeSpec:
    executable: Text  # TODO
    package: Optional[Text]
    name: Optional[Text]
    namespace: Optional[Text]
    exec_name: Optional[Text]
    params: List[Tuple[Text, Text]]
    params_files: List[Text]
    remaps: List[Tuple[Text, Text]]
    ros_args: Optional[List[Text]]
    args: Optional[List[Text]]
    cmd: List[Text]


def record_node(node: Node) -> NodeSpec:
    executable = record_substitution(node.node_executable)
    package = record_substitution(node.node_package)

    ros_args = None
    if node._Node__ros_arguments is not None:
        ros_args = [record_substitution(subst) for subst in node._Node__ros_arguments]

    args = None
    if node._Node__arguments is not None:
        args = [record_substitution(subst) for subst in node._Node__arguments]

    namespace = None
    if node.expanded_node_namespace != node.UNSPECIFIED_NODE_NAMESPACE:
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

    # # Extract parameters
    params_files = list()
    params = list()
    node_params = node._Node__expanded_parameter_arguments

    if node_params is not None:
        for entry, is_file in node_params:
            if is_file:
                path = entry
                with open(path, "r") as fp:
                    data = fp.read()
                    params_files.append(data)
            else:
                assert is_a(entry, Parameter)
                name, value = param_to_kv(entry)
                params.append((name, value))

    if node.expanded_remapping_rules is None:
        remaps = list()
    else:
        remaps = node.expanded_remapping_rules

    # Store a node record
    spec = NodeSpec(
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
    return spec
