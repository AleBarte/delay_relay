"""Launch the delay_relay_node with configurable parameters."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # ---- arguments -------------------------------------------------------
        DeclareLaunchArgument(
            "delay_sec",
            default_value="10.0",
            description="Message delay in seconds (>= 0)",
        ),
        DeclareLaunchArgument(
            "input_topic",
            default_value="/model/forward_velocity_controller/commands",
            description="Topic to subscribe to",
        ),
        DeclareLaunchArgument(
            "output_topic",
            default_value="/remote/forward_velocity_controller/commands",
            description="Topic to publish on",
        ),
        DeclareLaunchArgument(
            "queue_size",
            default_value="100",
            description="QoS queue depth",
        ),
        DeclareLaunchArgument(
            "timer_period_ms",
            default_value="2",
            description="Internal drain-timer period in milliseconds",
        ),

        # ---- node ------------------------------------------------------------
        Node(
            package="delay_relay",
            executable="delay_relay_node",
            name="delay_relay_node",
            output="screen",
            parameters=[{
                "delay_sec":       LaunchConfiguration("delay_sec"),
                "input_topic":     LaunchConfiguration("input_topic"),
                "output_topic":    LaunchConfiguration("output_topic"),
                "queue_size":      LaunchConfiguration("queue_size"),
                "timer_period_ms": LaunchConfiguration("timer_period_ms"),
            }],
        ),
    ])
