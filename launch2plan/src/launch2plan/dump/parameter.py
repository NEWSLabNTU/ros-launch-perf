from dataclasses import dataclass
from typing import Union, Text, List

from launch.utilities import is_a
from launch.utilities.typing_file_path import FilePath
from launch_ros.descriptions import ParameterValue, ParameterFile

from .substitution import (
    SubstitutionExpr,
    ParameterValueOrSubstitutionExpr,
    serialize_substitution,
    serialize_parameter_value_or_substitution,
)


ParameterValueType = Union[
    bool,
    int,
    float,
    Text,
    List[bool],
    List[int],
    List[float],
    List[Text],
]


@dataclass
class ParameterValueDump:
    value: ParameterValueOrSubstitutionExpr


@dataclass
class ParameterFileDump:
    param_file: Union[FilePath, SubstitutionExpr]
    allow_substs: Union[bool, SubstitutionExpr]


def serialize_parameter_value(param_value: ParameterValue) -> ParameterValueDump:
    value = serialize_parameter_value_or_substitution(
        param_value._ParameterValue__value
    )
    return ParameterValueDump(value=value)


def serialize_parameters_file(param_file: ParameterFile) -> ParameterFileDump:
    path = param_file._ParameterFile__param_file
    if is_a(path, list):
        path = serialize_substitution(path)
    else:
        assert is_a(path, FilePath)

    allow_substs = param_file._ParameterFile__allow_substs
    if is_a(allow_substs, list):
        allow_substs = serialize_substitution(path)
    else:
        assert is_a(allow_substs, bool)

    return ParameterFileDump(param_file=param_file, allow_substs=allow_substs)
