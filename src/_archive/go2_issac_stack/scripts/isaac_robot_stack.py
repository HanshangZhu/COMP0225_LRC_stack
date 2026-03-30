import os
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def build_isaac_robot_stack(
    ns,
    use_sim_time,
    robot_description,
    joints_config,
    links_config,
    gait_config,
    ekf_base_to_footprint,
    ekf_footprint_to_odom,
    condition=None,
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
        condition=condition,
    )

    effort_controller_name = f"{ns}_joint_group_effort_controller"
    # Isaac uses joint_command
    effort_topic = "joint_command"
    # wait, previously I used joint_trajectory_to_joint_state python node.
    # If I just make champ output to joint_command natively as JointTrajectory, and my bridge bridges it, or wait...
    # My bridge bridges joint_group_effort_controller/joint_trajectory to joint_command.
    
    # Actually wait! The bridge is listening to joint_group_effort_controller/joint_trajectory inside the namespace!
    # So effort_topic = f"joint_group_effort_controller/joint_trajectory"
    effort_topic = "joint_group_effort_controller/joint_trajectory"

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        namespace=ns,
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True}, # Fake hardware so we use JointCommands not raw PWM
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": effort_topic},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=tf_remaps + [("cmd_vel/smooth", "cmd_vel"), ("/cmd_vel/smooth", "cmd_vel"), ("joy", "joy"), ("/joy", "joy")],
        output="screen",
        condition=condition,
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
        condition=condition,
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
        condition=condition,
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
        condition=condition,
    )

    return [
        robot_state_publisher_node,
        quadruped_controller_node,
        state_estimator_node,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
    ]
