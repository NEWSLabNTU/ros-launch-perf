from typing import List, Dict, Optional
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from dataclasses import dataclass
from enum import Enum


class ProcessKind(Enum):
    COMPOSABLE_NODE_CONTAINER = "composable_node_container"
    LIFECYCLE_NODE = "lifecycle_node"
    NODE = "node"
    UNKNOWN = "unknown"


@dataclass
class ProcessInfo:
    kind: Text
    cmdline: List[Tuple[Text]]


@dataclass
class LaunchDump:
    process: List[ProcessInfo]
    file_data: Dict[Text, Text]
