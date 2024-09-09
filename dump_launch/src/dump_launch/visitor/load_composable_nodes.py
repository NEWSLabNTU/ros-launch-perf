from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple
from typing import Text, Optional

from rcl_interfaces.msg import Parameter
from launch.utilities import is_a_subclass
from launch.action import Action
from launch.launch_context import LaunchContext
from launch_ros.actions.load_composable_nodes import (
    LoadComposableNodes,
    get_composable_node_load_request,
)
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch.utilities import normalize_to_list_of_substitutions
from launch_ros.descriptions import ComposableNode
from launch_ros.ros_adapters import get_ros_node
from launch_ros.utilities import add_node_name
from launch_ros.utilities import evaluate_parameters
from launch_ros.utilities import get_node_name_count
from launch_ros.utilities import make_namespace_absolute
from launch_ros.utilities import prefix_namespace
from launch_ros.utilities import to_parameters_list
from launch_ros.utilities.normalize_parameters import normalize_parameter_dict
from launch.utilities import perform_substitutions
from launch.some_substitutions_type import SomeSubstitutionsType_types_tuple

import composition_interfaces.srv

from ..launch_dump import LaunchDump, LoadNodeRecord


def visit_load_composable_nodes(
    load: LoadComposableNodes, context: LaunchContext, dump: LaunchDump
)-> Optional[List[Action]]:
    # resolve target container node name
    target_container = load._LoadComposableNodes__target_container

    if is_a_subclass(target_container, ComposableNodeContainer):
        load._LoadComposableNodes__final_target_container_name = target_container.node_name
    elif isinstance(target_container, SomeSubstitutionsType_types_tuple):
        subs = normalize_to_list_of_substitutions(target_container)
        load._LoadComposableNodes__final_target_container_name = perform_substitutions(
            context, subs)
    else:
        load._LoadComposableNodes__logger.error(
            'target container is neither a ComposableNodeContainer nor a SubstitutionType')
        return

    # Create a client to load nodes in the target container.
    load._LoadComposableNodes__rclpy_load_node_client = get_ros_node(context).create_client(
        composition_interfaces.srv.LoadNode, '{}/_container/load_node'.format(
            load._LoadComposableNodes__final_target_container_name
        )
    )

    # Generate load requests before execute() exits to avoid race with context changing
    # due to scope change (e.g. if loading nodes from within a GroupAction).

    load_node_requests = list()
    for node_description in load._LoadComposableNodes__composable_node_descriptions:
        request = get_composable_node_load_request(node_description, context)
        load_node_requests.append(request)
        record = LoadNodeRecord(
            package=request.package_name,
            plugin=request.plugin_name,
            container_node_name=request.node_name,
            namespace=request.node_namespace,
            log_level=log_level_code_to_text(request.log_level),
            remaps=dict(text_to_kv(expr) for expr in request.remap_rules),
            parameters=dict(param_to_kv(param) for param in request.parameters),
            extra_arguments=dict(
                param_to_kv(param) for param in request.extra_arguments
            ),
        )
        dump.load_node.append(record)

    context.add_completion_future(
        context.asyncio_loop.run_in_executor(
            None, load._load_in_sequence, load_node_requests, context
        )
    )




def log_level_code_to_text(code: int) -> Optional[Text]:
    match code :
        case 0:
            return None
        case 10:
            return 'DEBUG'
        case 20:
            return 'INFO'
        case 30:
            return 'WARN'
        case 40:
            return 'ERROR'
        case 50:
            return 'FATAL'
        case _:
            raise ValueError(f"unknown log level code {code}")

def text_to_kv(expr: Text):
    return tuple(expr.split(":=", 1))

def param_to_kv(param: Parameter):
    pvalue = param.value

    match pvalue.type:
        case 1:
            value = pvalue.bool_value
        case 2:
            value = pvalue.integer_value
        case 3:
            value = pvalue.double_value
        case 4:
            value = pvalue.string_value
        case 5:
            # ipdb.set_trace()
            value = list(pvalue.byte_array_value)
        case 6:
            # ipdb.set_trace()
            value = list(pvalue.bool_array_value)
        case 7:
            # ipdb.set_trace()
            value = list(pvalue.integer_array_value)
        case 8:
            # ipdb.set_trace()
            value = list(pvalue.double_array_value)
        case 9:
            # ipdb.set_trace()
            value = list(pvalue.string_array_value)
        case _:
            raise ValueError(f"unknown parameter type code {param.type}")

    return param.name, value
