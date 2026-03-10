import importlib.util
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo


def _load_delegate_launch() -> LaunchDescription:
    mtare_share = get_package_share_directory("mtare_ros2")
    delegate_path = os.path.join(mtare_share, "launch", "dual_mtare_runtime.launch.py")

    spec = importlib.util.spec_from_file_location("mtare_ros2_dual_mtare_runtime", delegate_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"Unable to load delegated launch file: {delegate_path}")

    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "generate_launch_description"):
        raise RuntimeError(
            f"Delegated launch module '{delegate_path}' does not provide generate_launch_description()."
        )
    return module.generate_launch_description()


def generate_launch_description() -> LaunchDescription:
    delegated = _load_delegate_launch()

    ld = LaunchDescription()
    ld.add_action(
        LogInfo(
            msg=(
                "[DEPRECATED] go2_issac_stack/two_go2_isaac_mtare_bridge.launch.py is a compatibility wrapper. "
                "Use mtare_ros2/dual_mtare_runtime.launch.py directly."
            )
        )
    )
    for entity in delegated.entities:
        ld.add_action(entity)
    return ld
