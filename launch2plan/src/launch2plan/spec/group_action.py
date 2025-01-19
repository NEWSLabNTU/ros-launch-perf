from launch.actions.group_action import GroupAction
from launch.launch_context import LaunchContext

from .entity import record_entity
from ..launch_dump import LaunchDump


def record_group_action(
    group: GroupAction,
    context: LaunchContext,
    dump: LaunchDump,
):
    for entity in group.get_sub_entities():
        record_entity(entity)
