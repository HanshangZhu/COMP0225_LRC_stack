"""Reusable launch building blocks for go2_gazebo_sim stacks."""

import os
from xml.dom import minidom

from ament_index_python.packages import get_package_share_directory
from launch.actions import TimerAction
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def nav_profile_path(filename: str) -> str:
    return os.path.join(get_package_share_directory("go2_gazebo_sim"), "config", "nav", filename)


def build_rviz_node(rviz_file: str, use_sim_time, condition=None, namespace: str | None = None, name: str = "rviz2"):
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
    if condition is not None:
        kwargs["condition"] = condition
    return Node(**kwargs)


def build_wall_checker_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "wall_collision_checker"):
    params = [
        nav_profile_path("wall_checker.yaml"),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "wall_collision_checker.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_geometric_frontier_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "geometric_frontier",
    condition=None,
):
    params = [
        nav_profile_path(profile),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "geometric_frontier.py",
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


def build_reactive_nav_node(
    ns: str | None,
    use_sim_time,
    profile: str,
    extra_params=None,
    remappings=None,
    name: str = "reactive_nav",
):
    params = [
        nav_profile_path(profile),
        {"use_sim_time": use_sim_time},
    ]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "reactive_nav.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_goalpoint_bridge_node(ns: str | None, use_sim_time, remappings=None, name: str = "goalpoint_to_waypoint"):
    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "goalpoint_to_waypoint.py",
        "name": name,
        "parameters": [{"use_sim_time": use_sim_time}],
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_frontier_recovery_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "frontier_recovery"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "frontier_recovery.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_motion_monitor_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "motion_monitor"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "motion_monitor.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_autonomy_enabler_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "autonomy_enabler"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "autonomy_enabler.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_gazebo_frontier_visual_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "gazebo_frontier_visual", condition=None):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "gazebo_frontier_visual.py",
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


def build_pointcloud_to_laserscan_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "pointcloud_to_laserscan"):
    params = [{
        'use_sim_time': use_sim_time,
        'target_frame': 'base_link',  # Changed to base_link for local frame, or keep odom? Launch had odom.
        'transform_tolerance': 1.0,
        'min_height': 0.3,
        'max_height': 1.0,
        'angle_min': -3.14159,
        'angle_max': 3.14159,
        'angle_increment': 0.006135923151543,
        'scan_time': 0.1,
        'range_min': 0.2,
        'range_max': 20.0,
        'use_inf': True,
    }]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "pointcloud_to_laserscan",
        "executable": "pointcloud_to_laserscan_node",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def build_qos_bridge_node(ns: str | None, use_sim_time, extra_params=None, remappings=None, name: str = "qos_bridge"):
    params = [{"use_sim_time": use_sim_time}]
    if extra_params:
        params.append(extra_params)

    kwargs = {
        "package": "go2_gazebo_sim",
        "executable": "qos_bridge.py",
        "name": name,
        "parameters": params,
        "output": "screen",
    }
    if ns:
        kwargs["namespace"] = ns
    if remappings:
        kwargs["remappings"] = remappings
    return Node(**kwargs)


def _strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            _strip_comments(child)


def _child_elements(parent, name=None):
    for child in parent.childNodes:
        if child.nodeType != child.ELEMENT_NODE:
            continue
        if name is None or child.tagName == name:
            yield child


def _get_or_create_child(parent, name):
    for child in _child_elements(parent, name):
        return child
    child = parent.ownerDocument.createElement(name)
    parent.appendChild(child)
    return child


def _set_text(parent, value):
    for child in list(parent.childNodes):
        parent.removeChild(child)
    parent.appendChild(parent.ownerDocument.createTextNode(value))


def _rewrite_plugin_remap(remap_text, ns):
    text = remap_text.strip()
    if text == "odom:=odom/ground_truth":
        return f"odom:=/{ns}/odom/ground_truth"
    if text == "~/out:=scan":
        return f"~/out:=/{ns}/scan"
    if text == "~/out:=/registered_scan":
        return f"~/out:=/{ns}/registered_scan"
    return text


def build_namespaced_robot_description(robot_description, ns, ros_control_param_file):
    doc = minidom.parseString(robot_description)
    _strip_comments(doc)

    for plugin in doc.getElementsByTagName("plugin"):
        if not plugin.hasAttribute("filename"):
            continue

        filename = plugin.getAttribute("filename")
        if plugin.hasAttribute("name"):
            original_name = plugin.getAttribute("name")
            if original_name and not original_name.endswith(f"_{ns}"):
                plugin.setAttribute("name", f"{original_name}_{ns}")

        if "libgazebo_ros2_control.so" in filename:
            ros = _get_or_create_child(plugin, "ros")
            _set_text(_get_or_create_child(ros, "namespace"), f"/{ns}")
            _set_text(_get_or_create_child(plugin, "robotNamespace"), f"/{ns}")
            _set_text(_get_or_create_child(plugin, "parameters"), ros_control_param_file)
            continue

        ros = _get_or_create_child(plugin, "ros")
        if "imu_sensor" in filename:
            _set_text(_get_or_create_child(ros, "namespace"), f"/{ns}/imu")

        remaps = list(_child_elements(ros, "remapping"))
        for remap in remaps:
            current_text = "".join(node.data for node in remap.childNodes if node.nodeType == node.TEXT_NODE)
            new_text = _rewrite_plugin_remap(current_text, ns)
            if new_text != current_text.strip():
                _set_text(remap, new_text)

        existing_texts = {
            "".join(node.data for node in remap.childNodes if node.nodeType == node.TEXT_NODE).strip()
            for remap in remaps
        }

        if "libgazebo_ros_p3d.so" in filename and not any(text.startswith("odom:=") for text in existing_texts):
            remap = doc.createElement("remapping")
            _set_text(remap, f"odom:=/{ns}/odom/ground_truth")
            ros.appendChild(remap)

        if "libgazebo_ros_ray_sensor.so" in filename:
            target = f"~/out:=/{ns}/scan"
            if "3d_lidar" in plugin.getAttribute("name"):
                target = f"~/out:=/{ns}/registered_scan"
            if not any(text.startswith("~/out:=") for text in existing_texts):
                remap = doc.createElement("remapping")
                _set_text(remap, target)
                ros.appendChild(remap)

    return doc.documentElement.toxml()


def build_dual_robot_stack(
    *,
    ns,
    spawn_x,
    spawn_y,
    spawn_yaw,
    use_sim_time,
    robot_description,
    joints_config,
    links_config,
    gait_config,
    ekf_base_to_footprint,
    ekf_footprint_to_odom,
):
    tf_remaps = [("/tf", f"/{ns}/tf"), ("/tf_static", f"/{ns}/tf_static")]

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time},
        ],
        remappings=tf_remaps,
        output="screen",
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=tf_remaps + [("cmd_vel/smooth", "cmd_vel"), ("/cmd_vel/smooth", "cmd_vel"), ("joy", "joy"), ("/joy", "joy")],
        output="screen",
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": False},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=tf_remaps,
        output="screen",
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=ns,
        name="base_to_footprint_ekf",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            ekf_base_to_footprint,
        ],
        remappings=tf_remaps + [("odometry/filtered", "odom/local")],
        output="screen",
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        namespace=ns,
        name="footprint_to_odom_ekf",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            ekf_footprint_to_odom,
        ],
        remappings=tf_remaps + [("odometry/filtered", "odom")],
        output="screen",
    )

    spawn_entity_node = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=[
            "-entity",
            ns,
            "-topic",
            f"/{ns}/robot_description",
            "-robot_namespace",
            f"/{ns}",
            "-x",
            spawn_x,
            "-y",
            spawn_y,
            "-z",
            "0.45",
            "-R",
            "0",
            "-P",
            "0",
            "-Y",
            spawn_yaw,
        ],
    )

    load_joint_state_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_states_controller",
            "--controller-manager",
            f"/{ns}/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    load_joint_effort_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_group_effort_controller",
            "--controller-manager",
            f"/{ns}/controller_manager",
            "--controller-manager-timeout",
            "60",
        ],
        output="screen",
    )

    contact_sensor = Node(
        package="champ_gazebo",
        executable="contact_sensor",
        namespace=ns,
        parameters=[{"use_sim_time": use_sim_time}, links_config],
        output="screen",
    )

    stand_up_node = Node(
        package="go2_gazebo_sim",
        executable="stand_up_slowly.py",
        namespace=ns,
        remappings=[
            ("/joint_group_effort_controller/joint_trajectory", f"/{ns}/joint_group_effort_controller/joint_trajectory")
        ],
        output="screen",
    )

    return [
        robot_state_publisher_node,
        quadruped_controller_node,
        state_estimator_node,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        spawn_entity_node,
        TimerAction(period=6.0, actions=[load_joint_state_controller, load_joint_effort_controller]),
        contact_sensor,
        TimerAction(period=10.0, actions=[stand_up_node]),
    ]
