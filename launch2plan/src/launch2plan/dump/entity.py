from dataclasses import dataclass
from typing import Union

from launch import LaunchDescription
from launch.utilities import is_a
from launch.launch_description_entity import LaunchDescriptionEntity
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.opaque_function import OpaqueFunction
from launch.actions.group_action import GroupAction
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.node import Node
from launch_ros.actions.push_ros_namespace import PushRosNamespace
from launch_ros.actions.load_composable_nodes import LoadComposableNodes
from launch_ros.actions.set_use_sim_time import SetUseSimTime

from .node import serialize_node, NodeDump
from .declare_launch_argument import (
    serialize_declare_launch_argument,
    DeclareLaunchArgumentDump,
)
from .group_action import serialize_group_action, GroupActionDump
from .set_launch_configuration import (
    serialize_set_launch_configuration,
    SetLaunchConfigurationDump,
)
from .include_launch_description import IncludeLaunchDescriptionDump
from .composable_node import LoadComposableNodeDump, ComposableNodeContainerDump
from .set_use_sim_time import SetUseSimTimeDump, serialize_set_use_sim_time

ActionDump = Union[
    DeclareLaunchArgumentDump,
    SetLaunchConfigurationDump,
    GroupActionDump,
    IncludeLaunchDescriptionDump,
    ComposableNodeContainerDump,
    LoadComposableNodeDump,
    SetUseSimTimeDump,
    NodeDump,
]


def serialize_action(
    entity: LaunchDescriptionEntity,
    dump: "LaunchDescriptionDump",
) -> ActionDump:
    if is_a(entity, DeclareLaunchArgument):
        return serialize_declare_launch_argument(entity)

    elif is_a(entity, SetLaunchConfiguration):
        return serialize_set_launch_configuration(entity)

    elif is_a(entity, OpaqueFunction):
        # TODO
        pass

    elif is_a(entity, GroupAction):
        serialize_group_action(entity, dump)

    elif is_a(entity, IncludeLaunchDescription):
        # Late import due to circular import
        from .include_launch_description import serialize_include_launch_description

        serialize_include_launch_description(entity, dump)

    elif is_a(entity, ComposableNodeContainer):
        # TODO
        pass

    elif is_a(entity, LoadComposableNodes):
        # TODO
        pass

    elif is_a(entity, Node):
        return serialize_node(entity)

    elif is_a(entity, SetUseSimTime):
        return serialize_set_use_sim_time(entity)

    else:
        # The following entity types are excluded
        # - PushLaunchConfigurations
        # - PopLaunchConfigurations
        # - PushEnvironment
        # - PushRosNamespace
        # - PopEnvironment
        # - SetParameter
        # - SetParametersFromFile
        # - SetRemap
        raise ValueError(f"unknown entity type; {type(entity)}")
