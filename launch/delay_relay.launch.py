"""Launch the delay_relay_node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("delay_sec",               default_value="1.0"),
        DeclareLaunchArgument("input_topic",             default_value="/model/forward_velocity_controller/commands"),
        DeclareLaunchArgument("output_topic",            default_value="/remote/forward_velocity_controller/commands"),
        DeclareLaunchArgument("remote_joint_state_topic",default_value="/model/joint_states"),
        DeclareLaunchArgument("local_joint_state_topic", default_value="/remote/joint_states"),
        DeclareLaunchArgument("init_position_threshold", default_value="0.01"),
        DeclareLaunchArgument("init_timeout_sec",        default_value="10.0"),
        DeclareLaunchArgument("queue_size",              default_value="100"),
        DeclareLaunchArgument("timer_period_ms",         default_value="10"),

        Node(
            package="delay_relay",
            executable="delay_relay_node",
            name="delay_relay_node",
            output="screen",
            parameters=[{
                "delay_sec":                LaunchConfiguration("delay_sec"),
                "input_topic":              LaunchConfiguration("input_topic"),
                "output_topic":             LaunchConfiguration("output_topic"),
                "remote_joint_state_topic": LaunchConfiguration("remote_joint_state_topic"),
                "local_joint_state_topic":  LaunchConfiguration("local_joint_state_topic"),
                "init_position_threshold":  LaunchConfiguration("init_position_threshold"),
                "init_timeout_sec":         LaunchConfiguration("init_timeout_sec"),
                "queue_size":               LaunchConfiguration("queue_size"),
                "timer_period_ms":          LaunchConfiguration("timer_period_ms"),
            }],
        ),
    ])
