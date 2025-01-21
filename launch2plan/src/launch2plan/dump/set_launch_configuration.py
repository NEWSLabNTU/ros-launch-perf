from dataclasses import dataclass

from launch.actions.set_launch_configuration import SetLaunchConfiguration

from .substitution import serialize_substitution, SubstitutionExpr


@dataclass
class SetLaunchConfigurationDump:
    name: SubstitutionExpr
    value: SubstitutionExpr


def serialize_set_launch_configuration(
    action: SetLaunchConfiguration,
) -> SetLaunchConfigurationDump:
    name = serialize_substitution(action.name)
    value = serialize_substitution(action.value)
    return SetLaunchConfigurationDump(name=name, value=value)
