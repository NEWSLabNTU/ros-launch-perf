from typing import Text, List, Dict, Optional
from dataclasses import dataclass

from launch.utilities import is_a
from launch.launch_description import LaunchDescription
from launch.launch_description_source import LaunchDescriptionSource
from launch.actions.push_launch_configurations import PushLaunchConfigurations
from launch.actions.pop_launch_configurations import PopLaunchConfigurations
from launch.actions.push_environment import PushEnvironment
from launch.actions.pop_environment import PopEnvironment
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions.push_ros_namespace import PushRosNamespace

from .entity import serialize_action
from .substitution import serialize_substitution, SubstitutionExpr


@dataclass
class LaunchDescriptionDump:
    file_path: Text
    push_namespace: Optional[SubstitutionExpr]
    action_list: List["ActionDump"]


def serialize_launch_description(
    desc: LaunchDescription,
    source: LaunchDescriptionSource,
) -> LaunchDescriptionDump:
    file_path = source.location
    dump = LaunchDescriptionDump(
        file_path=file_path, push_namespace=None, action_list=list()
    )

    for entity in desc.entities:
        # Check non-action cases
        if is_a(entity, PushRosNamespace):
            namespace = serialize_substitution(entity.namespace)

            if dump.push_namespace is None:
                dump.push_namespace = namespace
            else:
                raise ValueError("PushRosNamespace is done more than once")

        # Process the action
        else:
            action = serialize_action(entity, dump)
            dump.action_list.append(action)

    return dump
