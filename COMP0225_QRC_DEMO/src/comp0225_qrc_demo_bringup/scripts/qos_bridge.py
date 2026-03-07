#!/usr/bin/env python3
"""Compatibility wrapper.

Deprecated path kept for one release cycle.
"""

import os
import runpy
import sys


_REL_IMPL = "perception/qos_bridge.py"


def _resolve_impl() -> str:
    here = os.path.dirname(__file__)
    candidates = [
        os.path.normpath(os.path.join(here, _REL_IMPL)),
        os.path.normpath(os.path.join(here, "scripts", _REL_IMPL)),
    ]
    for path in candidates:
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(f"Could not resolve implementation for {__file__}: tried {candidates}")


def main() -> None:
    impl = _resolve_impl()
    print(
        "[DEPRECATED] go2_gazebo_sim/qos_bridge.py -> go2_gazebo_sim/scripts/perception/qos_bridge.py",
        file=sys.stderr,
    )
    sys.path.insert(0, os.path.dirname(impl))
    runpy.run_path(impl, run_name="__main__")


if __name__ == "__main__":
    main()
