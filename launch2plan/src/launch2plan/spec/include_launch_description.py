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
from .entity import record_entity

# @dataclass
# class LaunchDescriptionSpec:
#     pass


def record_include_launch_description(
    include: IncludeLaunchDescription,
    context: LaunchContext,
    dump: LaunchDump,
):
    source = include.launch_description_source
    launch_description = source.get_launch_description(context)
    file_path = source.location
    # print("#########", file_path)

    for entity in launch_description.entities:
        record_entity(entity)
