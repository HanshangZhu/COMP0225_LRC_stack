#!/usr/bin/env python3
"""Compatibility wrapper.

Deprecated path kept for one release cycle.
"""

import os
import runpy
import sys


def _candidate_paths() -> list[str]:
    here = os.path.dirname(__file__)
    return [
        os.path.normpath(os.path.join(here, "..", "go2_nav_algorithms", "geometric_frontier.py")),
        os.path.normpath(os.path.join(here, "..", "..", "go2_nav_algorithms", "scripts", "geometric_frontier.py")),
    ]


def _resolve_impl() -> str:
    for path in _candidate_paths():
        if os.path.isfile(path):
            return path
    raise FileNotFoundError("go2_nav_algorithms geometric_frontier.py not found in expected locations")


def main() -> None:
    impl = _resolve_impl()
    print(
        "[DEPRECATED] go2_gazebo_sim/geometric_frontier.py -> go2_nav_algorithms/geometric_frontier.py",
        file=sys.stderr,
    )
    sys.path.insert(0, os.path.dirname(impl))
    runpy.run_path(impl, run_name="__main__")


if __name__ == "__main__":
    main()
