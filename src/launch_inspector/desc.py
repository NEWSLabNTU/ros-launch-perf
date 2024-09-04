from typing import List
from typing import Optional
from typing import Text  # noqa: F401
from typing import Tuple  # noqa: F401
from dataclasses import dataclass

@dataclass
class NodeDesc:
    package: str
    executable: str
    name: str
    namespace: str
    remapping: Optional[List[Tuple[Text, Text]]]
