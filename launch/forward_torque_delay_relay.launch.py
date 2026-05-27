"""Launch the forward_torque_delay_relay_node for Franka joint-state PD tracking."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("delay_sec", default_value="5.0"),
        DeclareLaunchArgument(
            "reference_joint_state_topic", default_value="/model/joint_states"),
        DeclareLaunchArgument(
            "output_topic", default_value="/remote/forward_torque_controller/commands"),
        DeclareLaunchArgument(
            "remote_joint_state_topic", default_value="/remote/joint_states"),
        DeclareLaunchArgument(
            "local_joint_state_topic", default_value="/model/joint_states"),
        DeclareLaunchArgument(
            "local_torque_cmd_topic", default_value="/model/forward_torque_controller/commands"),
        DeclareLaunchArgument(
            "p_gains",
            default_value="[20.0, 20.0, 20.0, 20.0, 10.0, 10.0, 5.0]"),
        DeclareLaunchArgument(
            "d_gains",
            default_value="[5.0, 5.0, 5.0, 5.0, 2.0, 2.0, 2.0]"),
        DeclareLaunchArgument("torque_limit", default_value="10.0"),
        DeclareLaunchArgument("init_position_threshold", default_value="0.01"),
        DeclareLaunchArgument("init_timeout_sec", default_value="10.0"),
        DeclareLaunchArgument("queue_size", default_value="100"),
        DeclareLaunchArgument("timer_period_ms", default_value="1"),
        DeclareLaunchArgument(
            "ordered_joint_names",
            default_value="['fer_joint1','fer_joint2','fer_joint3','fer_joint4','fer_joint5','fer_joint6','fer_joint7']"),

        Node(
            package="delay_relay",
            executable="forward_torque_delay_relay_node",
            name="forward_torque_delay_relay_node",
            output="screen",
            parameters=[{
                "delay_sec": LaunchConfiguration("delay_sec"),
                "reference_joint_state_topic": LaunchConfiguration("reference_joint_state_topic"),
                "output_topic": LaunchConfiguration("output_topic"),
                "remote_joint_state_topic": LaunchConfiguration("remote_joint_state_topic"),
                "local_joint_state_topic": LaunchConfiguration("local_joint_state_topic"),
                "local_torque_cmd_topic": LaunchConfiguration("local_torque_cmd_topic"),
                "p_gains": ParameterValue(LaunchConfiguration("p_gains"), value_type=list[float]),
                "d_gains": ParameterValue(LaunchConfiguration("d_gains"), value_type=list[float]),
                "torque_limit": LaunchConfiguration("torque_limit"),
                "init_position_threshold": LaunchConfiguration("init_position_threshold"),
                "init_timeout_sec": LaunchConfiguration("init_timeout_sec"),
                "queue_size": LaunchConfiguration("queue_size"),
                "timer_period_ms": LaunchConfiguration("timer_period_ms"),
                "ordered_joint_names": LaunchConfiguration("ordered_joint_names"),
            }],
        ),
    ])
