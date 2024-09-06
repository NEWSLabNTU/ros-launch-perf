"""Module for utility functions."""

import asyncio
from typing import List, Text, Optional
from typing import Tuple
from typing import cast

from launch.utilities import is_a
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from launch_ros.actions.load_composable_nodes import (
    LoadComposableNodes,
    get_composable_node_load_request,
)
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.lifecycle_node import LifecycleNode
from launch_ros.actions.node import Node
from composition_interfaces.srv import LoadNode
from rcl_interfaces.msg import Parameter, ParameterValue

from .dump import LaunchDump, LoadNodeRecord


def visit_recursive(
    entity: LaunchDescriptionEntity, context: LaunchContext, dump: LaunchDump
) -> List[Tuple[LaunchDescriptionEntity, asyncio.Future]]:
    """
    Visit given entity, as well as all sub-entities, and collect any futures.

    Sub-entities are visited recursively and depth-first.
    The future is collected from each entity (unless it returns None) before
    continuing on to more sub-entities.

    This function may call itself to traverse the sub-entities recursively.
    """

    sub_entities = entity.visit(context)
    entity_future = entity.get_asyncio_future()

    if is_a(entity, LoadComposableNodes):
        load = cast(LoadComposableNodes, entity)
        on_load_composable_nodes(load, context, dump)

    elif is_a(entity, ComposableNodeContainer):
        container = cast(ComposableNodeContainer, entity)
        on_composable_node_container(container, context, dump)

    elif is_a(entity, LifecycleNode):
        lifecycle_node = cast(LifecycleNode, entity)
        on_lifecycle_node(lifecycle_node, context, dump)

    elif is_a(entity, Node):
        node = cast(Node, entity)
        on_node(node, context, dump)

    futures_to_return = []
    if entity_future is not None:
        futures_to_return.append((entity, entity_future))
    if sub_entities is not None:
        for sub_entity in sub_entities:
            futures_to_return += visit_recursive(sub_entity, context, dump)
    return [
        future_pair for future_pair in futures_to_return if future_pair[1] is not None
    ]


def on_lifecycle_node(node: LifecycleNode, context: LaunchContext, dump: LaunchDump):
    pass


def on_composable_node_container(
    node: ComposableNodeContainer, context: LaunchContext, dump: LaunchDump
):
    pass


def on_node(node: Node, context: LaunchContext, dump: LaunchDump):
    pass


def on_load_composable_nodes(
    load: LoadComposableNodes, context: LaunchContext, dump: LaunchDump
):
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


    for node_description in load._LoadComposableNodes__composable_node_descriptions:
        request = get_composable_node_load_request(node_description, context)
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
