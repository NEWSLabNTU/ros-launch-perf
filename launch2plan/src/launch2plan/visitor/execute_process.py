from typing import Iterable
from typing import List
from typing import Optional
from typing import Text
from typing import Tuple

from launch.actions import ExecuteProcess
from launch.launch_context import LaunchContext
from launch.launch_description_entity import LaunchDescriptionEntity

from .execute_local import visit_execute_local


def visit_execute_process(
    process: ExecuteProcess, context: LaunchContext
) -> Optional[List[LaunchDescriptionEntity]]:
    visit_execute_local(process, context)
