"""Module for utility functions."""


import asyncio
from typing import List
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


def visit_recursive(
    entity: LaunchDescriptionEntity, context: LaunchContext
) -> List[Tuple[LaunchDescriptionEntity, asyncio.Future]]:
    """
    Visit given entity, as well as all sub-entities, and collect any futures.

    Sub-entities are visited recursively and depth-first.
    The future is collected from each entity (unless it returns None) before
    continuing on to more sub-entities.

    This function may call itself to traverse the sub-entities recursively.
    """

    if is_a(entity, LoadComposableNodes):
        load = cast(LoadComposableNodes, entity)
        load_node_requests = [
            get_composable_node_load_request(node_description, context)
            for node_description in load._LoadComposableNodes__composable_node_descriptions
        ]
        # TODO: store load_node_request information

    elif is_a(entity, ComposableNodeContainer):
        container = cast(ComposableNodeContainer, entity)

    elif is_a(entity, LifecycleNode):
        lifecycle_node = cast(LifecycleNode, entity)

    elif is_a(entity, Node):
        node = cast(Node, entity)

    sub_entities = entity.visit(context)
    entity_future = entity.get_asyncio_future()
    futures_to_return = []
    if entity_future is not None:
        futures_to_return.append((entity, entity_future))
    if sub_entities is not None:
        for sub_entity in sub_entities:
            futures_to_return += visit_recursive(sub_entity, context)
    return [
        future_pair for future_pair in futures_to_return if future_pair[1] is not None
    ]
