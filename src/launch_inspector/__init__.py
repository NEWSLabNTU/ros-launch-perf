import json
from collections import OrderedDict
import argparse
from typing import List  # noqa: F401
from typing import Set  # noqa: F401
from typing import Text
from typing import Tuple  # noqa: F401

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource

from .inspector import LaunchInspector


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("LAUNCH_FILE")
    parser.add_argument("LAUNCH_ARGS", nargs="*")
    parser.add_argument("--debug", action="store_true")
    parser.add_argument("-o", "--output")
    args = parser.parse_args()

    inspector = LaunchInspector(
        argv=args.LAUNCH_ARGS, noninteractive=True, debug=args.debug
    )

    parsed_args = parse_launch_arguments(args.LAUNCH_ARGS)
    launch_description = LaunchDescription(
        [
            IncludeLaunchDescription(
                AnyLaunchDescriptionSource(args.LAUNCH_FILE),
                launch_arguments=parsed_args,
            )
        ]
    )
    inspector.include_launch_description(launch_description)

    inspector.run(shutdown_when_idle=True)
    dump = inspector.dump()

    if args.output is not None:
        with open(args.output, "w") as fp:
            json.dump(dump, fp, sort_keys=True, indent=4)
    else:
        print(json.dumps(dump, sort_keys=True, indent=4))

    return 0


def parse_launch_arguments(launch_arguments: List[Text]) -> List[Tuple[Text, Text]]:
    """Parse the given launch arguments from the command line, into list of tuples for launch."""
    parsed_launch_arguments = OrderedDict()  # type: ignore
    for argument in launch_arguments:
        count = argument.count(":=")
        if (
            count == 0
            or argument.startswith(":=")
            or (count == 1 and argument.endswith(":="))
        ):
            raise RuntimeError(
                "malformed launch argument '{}', expected format '<name>:=<value>'".format(
                    argument
                )
            )
        name, value = argument.split(":=", maxsplit=1)
        parsed_launch_arguments[name] = value  # last one wins is intentional
    return parsed_launch_arguments.items()
