from dataclasses import dataclass

from launch import LaunchDescription
from launch.utilities import is_a
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.opaque_function import OpaqueFunction
from launch.actions.group_action import GroupAction
from launch.actions.include_launch_description import IncludeLaunchDescription
from launch_ros.actions.composable_node_container import ComposableNodeContainer
from launch_ros.actions.node import Node
from launch_ros.actions.push_ros_namespace import PushRosNamespace
from launch_ros.actions.load_composable_nodes import LoadComposableNodes


@dataclass
class IncludeLaunchDescriptionDump:
    pass


def serialize_include_launch_description(
    include: IncludeLaunchDescription,
    dump: "LaunchDescriptionDump",
) -> IncludeLaunchDescriptionDump:
    # TODO
    pass
