from dataclasses import dataclass
from typing import List, Optional

from launch.utilities import is_a
from launch.actions.group_action import GroupAction
from launch.actions.push_launch_configurations import PushLaunchConfigurations
from launch.actions.pop_launch_configurations import PopLaunchConfigurations
from launch.actions.reset_launch_configurations import ResetLaunchConfigurations
from launch.actions.push_environment import PushEnvironment
from launch.actions.pop_environment import PopEnvironment
from launch.actions.reset_environment import ResetEnvironment
from launch.actions.set_launch_configuration import SetLaunchConfiguration
from launch.actions.declare_launch_argument import DeclareLaunchArgument
from launch_ros.actions.set_parameter import SetParameter
from launch_ros.actions.set_parameters_from_file import SetParametersFromFile
from launch_ros.actions.set_remap import SetRemap
from launch_ros.actions.push_ros_namespace import PushRosNamespace

from .substitution import serialize_substitution, SubstitutionExpr


@dataclass
class GroupActionDump:
    scoped: bool
    forwarding: bool
    push_namespace: Optional[SubstitutionExpr]
    action_list: List["ActionDump"]


def serialize_group_action(
    group: GroupAction,
    dump: "LaunchDescriptionDump",
) -> GroupActionDump:
    all_entities = group.get_sub_entities()

    # Determine "scoped" and "forwarding" properties
    if len(all_entities) >= 4:
        if is_a(all_entities[0], PushLaunchConfigurations):
            assert is_a(all_entities[1], PushEnvironment)
            assert is_a(all_entities[-2], PopEnvironment)
            assert is_a(all_entities[-1], PopLaunchConfigurations)
            scoped = True

            if is_a(all_entities[2], ResetEnvironment):
                assert is_a(all_entities[3], ResetLaunchConfigurations)
                forwarding = False
                action_entities = all_entities[4:-2]
            else:
                forwarding = True
                action_entities = all_entities[2:-2]

        else:
            scoped = False
            forwarding = False
            action_entities = all_entities
    else:
        scoped = False
        forwarding = False
        action_entities = all_entities

    # Process action entities
    push_namespace = None
    action_list = list()

    for entity in action_entities:
        from .entity import serialize_action  # Due to circular import

        # Check actions that are specific to groups
        if is_a(entity, SetParameter):
            # TODO
            pass

        elif is_a(entity, SetParametersFromFile):
            # TODO
            pass

        elif is_a(entity, SetRemap):
            # TODO
            pass

        elif is_a(entity, PushRosNamespace):
            if push_namespace is not None:
                raise ValueError("PushRosNamespace is done more than once")
            if not scoped:
                raise ValueError(
                    "PushRosNamespace in non-scoped group is not supported"
                )
            push_namespace = serialize_substitution(entity.namespace)

        # Process the action
        else:
            action = serialize_action(entity, dump)
            action_list.append(action)

    return GroupActionDump(
        scoped=scoped,
        forwarding=forwarding,
        push_namespace=push_namespace,
        action_list=action_list,
    )
