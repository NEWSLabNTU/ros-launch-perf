from launch.actions.declare_launch_argument import DeclareLaunchArgument
from .substitution import record_substitution


def record_declare_launch_argument(entity: DeclareLaunchArgument):
    name = entity.name

    default_value = None
    if entity.default_value is not None:
        default_value = record_substitution(entity.default_value)

    description = entity.description
    choices = entity.choices
