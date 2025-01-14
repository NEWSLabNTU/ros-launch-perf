from dataclasses import dataclass

from launch import LaunchDescription
from launch.utilities import is_a
from launch.launch_context import LaunchContext
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.opaque_function import OpaqueFunction
from launch.actions.group_action import GroupAction
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.node import Node
from launch_ros.actions.push_ros_namespace import PushRosNamespace
from launch_ros.actions.load_composable_nodes import LoadComposableNodes

from ..launch_dump import LaunchDump, LaunchDescriptionRecord
from .node import record_node
from .declare_launch_argument import record_declare_launch_argument


# @dataclass
# class LaunchDescriptionSpec:
#     pass


def record_launch_description(
    action: IncludeLaunchDescription,
    desc: LaunchDescription,
    context: LaunchContext,
    dump: LaunchDump,
):
    source = action.launch_description_source
    file_path = source.location

    print("#########", file_path)

    node = list()

    for entity in desc.entities:
        if is_a(entity, DeclareLaunchArgument):
            record_declare_launch_argument(entity)

        elif is_a(entity, SetLaunchConfiguration):
            # TODO
            pass

        elif is_a(entity, OpaqueFunction):
            # TODO
            pass

        elif is_a(entity, GroupAction):
            # TODO
            pass

        elif is_a(entity, IncludeLaunchDescription):
            # TODO
            pass

        elif is_a(entity, ComposableNodeContainer):
            # TODO
            pass

        elif is_a(entity, Node):
            record_node(entity)

        elif is_a(entity, PushRosNamespace):
            # TODO
            pass

        elif is_a(entity, LoadComposableNodes):
            # TODO
            pass

        else:
            print(entity)
