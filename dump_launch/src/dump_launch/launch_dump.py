from typing import List, Dict, Optional, Any
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from dataclasses import dataclass


@dataclass
class NodeRecord:
    executable: Text
    package: Optional[Text]
    name: Optional[Text]
    namespace: Optional[Text]
    exec_name: Optional[Text]
    params: List[Tuple[Text, Text]]
    params_files: List[Text]
    remaps: List[Tuple[Text, Text]]
    ros_args: Optional[List[Text]]
    args: Optional[List[Text]]
    cmd: List[Text]


@dataclass
class LoadNodeRecord:
    package: Text
    plugin: Text
    target_container_name: Text
    node_name: Text
    namespace: Text
    log_level: Optional[Text]
    remaps: List[Tuple[Text, Text]]
    params: List[Tuple[Text, Text]]
    extra_args: Dict[Text, Text]


@dataclass
class ComposableNodeContainerRecord:
    name: Text
    namespace: Text


@dataclass
class LaunchDump:
    node: List[NodeRecord]
    load_node: List[LoadNodeRecord]
    container: List[ComposableNodeContainerRecord]
    lifecycle_node: List[Text]
    file_data: Dict[Text, Text]
