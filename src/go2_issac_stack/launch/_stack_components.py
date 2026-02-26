"""Reusable launch building blocks for go2_issac_stack stacks."""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

sys.path.append(os.path.join(get_package_share_directory("go2_nav_algorithms"), "launch"))
from pipeline_components import build_pointcloud_to_laserscan_node as build_pointcloud_to_laserscan_node_planner


def nav_profile_path(filename: str) -> str:
    return os.path.join(get_package_share_directory("go2_issac_stack"), "config", "nav", filename)


def _make_node(
    package: str,
    executable: str,
    name: str,
    use_sim_time,
    ns: str | None = None,
    base_params=None,
    extra_params=None,
    remappings=None,
    condition=None,
):
    params = []
    if base_params is not None:
        params.append(base_params)
    params.append({"use_sim_time": use_sim_time})
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": package,
        "executable": executable,
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_rviz_node(
    rviz_file: str,
    use_sim_time,
    condition=None,
    namespace: str | None = None,
    remappings=None,
    name: str = "rviz2",
):
    kwargs = {
        "package": "rviz2",
        "executable": "rviz2",
        "arguments": ["-d", rviz_file],
        "parameters": [{"use_sim_time": use_sim_time}],
        "output": "screen",
        "name": name,
    }
    if namespace:
        kwargs["namespace"] = namespace
    if remappings:
        kwargs["remappings"] = remappings
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_qos_bridge_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "qos_bridge"):
    return _make_node(
        package="go2_gazebo_sim",
        executable="qos_bridge.py",
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params=extra_params,
        remappings=remappings,
    )


def build_geometric_frontier_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "geometric_frontier",
    condition=None,
):
    # Backward-compatible builder name. The implementation now uses the
    # streamlined map-based explorer in go2_issac_stack.
    return _make_node(
        package="go2_nav_algorithms",
        executable="simple_frontier_explorer.py",
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        base_params=nav_profile_path(profile),
        extra_params=extra_params,
        remappings=remappings,
        condition=condition,
    )


def build_simple_scan_mapper_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "simple_scan_mapper",
    executable: str = "simple_scan_mapper_cpp",
    condition=None,
):
    return _make_node(
        package="go2_nav_algorithms",
        executable=executable,
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        base_params=nav_profile_path(profile),
        extra_params=extra_params,
        remappings=remappings,
        condition=condition,
    )


def build_reactive_nav_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "reactive_nav",
):
    return _make_node(
        package="go2_gazebo_sim",
        executable="reactive_nav.py",
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        base_params=nav_profile_path(profile),
        extra_params=extra_params,
        remappings=remappings,
    )


def build_autonomy_enabler_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "autonomy_enabler"):
    return _make_node(
        package="go2_gazebo_sim",
        executable="autonomy_enabler.py",
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params=extra_params,
        remappings=remappings,
    )


def build_pointcloud_to_laserscan_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "pointcloud_to_laserscan"):
    return build_pointcloud_to_laserscan_node_planner(
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params=extra_params,
        remappings=remappings,
        name=name,
    )


def build_isaac_topic_router_node(
    ns: str | None,
    use_sim_time,
    extra_params=None,
    remappings=None,
    name: str = "isaac_topic_router",
):
    return _make_node(
        package="go2_issac_stack",
        executable="isaac_topic_router.py",
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params=extra_params,
        remappings=remappings,
    )


def build_odom_tf_broadcaster_node(
    ns: str | None,
    use_sim_time,
    extra_params=None,
    remappings=None,
    name: str = "odom_tf_broadcaster",
):
    return _make_node(
        package="go2_issac_stack",
        executable="odom_tf_broadcaster.py",
        name=name,
        ns=ns,
        use_sim_time=use_sim_time,
        extra_params=extra_params,
        remappings=remappings,
    )
