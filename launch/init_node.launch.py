"""Launch the init_node for local/remote arm initialization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "remote_joint_state_topic", default_value="/remote/joint_states"),
        DeclareLaunchArgument(
            "local_joint_state_topic",  default_value="/model/joint_states"),
        DeclareLaunchArgument(
            "velocity_cmd_topic", default_value="/model/forward_velocity_controller/commands"),
        DeclareLaunchArgument("p_gain",             default_value="2.0"),
        DeclareLaunchArgument("velocity_limit",     default_value="0.5"),
        DeclareLaunchArgument("position_threshold", default_value="0.01"),
        DeclareLaunchArgument("timer_period_ms",    default_value="10"),

        Node(
            package="delay_relay",
            executable="init_node",
            name="init_node",
            output="screen",
            parameters=[{
                "remote_joint_state_topic": LaunchConfiguration("remote_joint_state_topic"),
                "local_joint_state_topic":  LaunchConfiguration("local_joint_state_topic"),
                "velocity_cmd_topic":       LaunchConfiguration("velocity_cmd_topic"),
                "p_gain":                   LaunchConfiguration("p_gain"),
                "velocity_limit":           LaunchConfiguration("velocity_limit"),
                "position_threshold":       LaunchConfiguration("position_threshold"),
                "timer_period_ms":          LaunchConfiguration("timer_period_ms"),
            }],
        ),
    ])
