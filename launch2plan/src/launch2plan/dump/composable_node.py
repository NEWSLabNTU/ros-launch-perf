from typing import List, Dict, Optional, Any
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from dataclasses import dataclass


@dataclass
class LoadComposableNodeDump:
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
class ComposableNodeContainerDump:
    name: Text
    namespace: Text
