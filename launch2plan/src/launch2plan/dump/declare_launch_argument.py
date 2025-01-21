from typing import Text, List
from dataclasses import dataclass

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from .substitution import serialize_substitution, SubstitutionExpr


@dataclass
class DeclareLaunchArgumentDump:
    name: Text
    default_value: SubstitutionExpr
    description: Text
    choices: List[Text]


def serialize_declare_launch_argument(
    entity: DeclareLaunchArgument,
) -> DeclareLaunchArgumentDump:
    name = entity.name

    default_value = None
    if entity.default_value is not None:
        default_value = serialize_substitution(entity.default_value)

    description = entity.description
    choices = entity.choices

    return DeclareLaunchArgumentDump(name, default_value, description, choices)
