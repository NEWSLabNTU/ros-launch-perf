from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple

from launch.actoins.ExecuteProcess
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity
from ..dump import LaunchDump

from .execute_local import visit_execute_local


def visit_execute_process(
    process: ExecuteProcess, context: LaunchContext, dump: LaunchDump
) -> Optional[List[LaunchDescriptionEntity]]:
    visit_execute_local(process)
