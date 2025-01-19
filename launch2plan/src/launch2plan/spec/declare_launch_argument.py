from typing import Text
from dataclasses import dataclass

from launch.actions.declare_launch_argument import DeclareLaunchArgument
from .substitution import record_substitution

from .substitution import SubstitutionExpr


@dataclass
class DeclareLaunchArgumentSpec:
    name: Text
    default_value: SubstitutionExpr
    description: Text
    choices: List[Text]


def record_declare_launch_argument(
    entity: DeclareLaunchArgument,
) -> DeclareLaunchArgumentSpec:
    name = entity.name

    default_value = None
    if entity.default_value is not None:
        default_value = record_substitution(entity.default_value)

    description = entity.description
    choices = entity.choices

    return DeclareLaunchArgumentSpec(name, default_value, description, choices)
