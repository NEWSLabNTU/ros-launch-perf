from dataclasses import dataclass
from typing import List, Dict, Optional

@dataclass
class CommandLine:
    command: str
    user_args: List[str]
    remaps: Dict[str, str]
    params: Dict[str, str]
    params_files: List[str]
    log_level: Optional[str]
    log_config_file: Optional[str]
    enable_rosout_logs: Optional[bool]
    enable_stdout_logs: Optional[bool]
    enclave: Optional[str]
