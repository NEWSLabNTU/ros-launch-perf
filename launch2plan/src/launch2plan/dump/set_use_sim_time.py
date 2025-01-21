from dataclasses import dataclass

from launch_ros.actions.set_use_sim_time import SetUseSimTime


@dataclass
class SetUseSimTimeDump:
    value: bool

def serialize_set_use_sim_time(action: SetUseSimTime) -> SetUseSimTimeDump:
    return SetUseSimTimeDump(value=action.value)
