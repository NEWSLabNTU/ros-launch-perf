from dataclasses import dataclass
from typing import List, Optional, Text, Tuple, Dict

from launch.utilities import is_a
from launch_ros.actions.node import Node
from launch_ros.descriptions import Parameter, ParameterFile

from .substitution import (
    SubstitutionExpr,
    TextOrSubstitutionExpr,
    ParameterValueOrSubstitutionExpr,
    SubstitutionExpr,
    serialize_substitution,
    serialize_text_or_substitution,
)
from .parameter import (
    ParameterFileDump,
    serialize_parameter_value,
    serialize_parameters_file,
)
from ..utils import param_to_kv


@dataclass
class NodeDump:
    executable: TextOrSubstitutionExpr
    package: TextOrSubstitutionExpr
    name: Optional[TextOrSubstitutionExpr]
    namespace: Optional[TextOrSubstitutionExpr]
    params: List[Tuple[SubstitutionExpr, ParameterValueOrSubstitutionExpr]]
    params_files: List[ParameterFileDump]
    remaps: List[Tuple[Text, Text]]
    ros_args: Optional[List[SubstitutionExpr]]
    args: Optional[List[SubstitutionExpr]]


def serialize_node(node: Node) -> NodeDump:
    executable = serialize_text_or_substitution(node._Node__node_executable)
    package = serialize_text_or_substitution(node._Node__package)

    name = None
    if node._Node__node_name is not None:
        name = serialize_text_or_substitution(node._Node__node_name)

    namespace = None
    if node._Node__node_namespace is not None:
        namespace = serialize_text_or_substitution(node._Node__node_namespace)

    ros_args = None
    if node._Node__ros_arguments is not None:
        ros_args = [
            serialize_substitution(subst) for subst in node._Node__ros_arguments
        ]

    args = None
    if node._Node__arguments is not None:
        args = [serialize_substitution(subst) for subst in node._Node__arguments]

    # # Extract parameters
    params = list()
    params_files = list()

    for entry in node._Node__parameters:
        if is_a(entry, ParameterFile):
            params_file = serialize_parameters_file(entry)
            params_files.append(params_file)
        else:
            assert is_a(entry, dict)
            for param_name, value in entry.items():
                param_name = serialize_substitution(param_name)
                param_value = serialize_parameter_value(value)
                params.append((param_name, param_value))

    remaps = node._Node__remappings

    # Store a node record
    dump = NodeDump(
        executable=executable,
        package=package,
        name=name,
        namespace=namespace,
        remaps=remaps,
        params=params,
        params_files=params_files,
        ros_args=ros_args,
        args=args,
    )
    return dump
