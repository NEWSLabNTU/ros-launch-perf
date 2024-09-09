from typing import List, Dict, Optional, Any
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from dataclasses import dataclass
from enum import Enum


# class ProcessKind(Enum):
#     COMPOSABLE_NODE_CONTAINER = "composable_node_container"
#     LIFECYCLE_NODE = "lifecycle_node"
#     NODE = "node"
#     UNKNOWN = "unknown"


# @dataclass
# class ProcessRecord:
#     kind: Text
#     cmdline: List[Tuple[Text]]


@dataclass
class NodeRecord:
    executable: Text
    package: Optional[Text]
    name: Optional[Text]
    namespace: Optional[Text]
    exec_name: Optional[Text]
    params: Dict[Text, Any]
    params_files: List[Text]
    remaps: Dict[Text, Text]
    ros_args: Optional[List[Text]]
    args: Optional[List[Text]]
    cmd: List[Text]

    
@dataclass
class LoadNodeRecord:
    package: Text
    plugin: Text
    container_node_name: Text
    namespace: Text
    log_level: Optional[Text]
    remaps: Dict[Text, Text]
    parameters: Dict[Text, Text]
    extra_arguments: Dict[Text, Text]


@dataclass
class LaunchDump:
    node: List[NodeRecord]
    # process: List[ProcessRecord]
    load_node: List[LoadNodeRecord]
    file_data: Dict[Text, Text]
