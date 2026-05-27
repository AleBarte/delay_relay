"""Launch the forward_torque_init_node for Franka/model initialization."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "remote_joint_state_topic", default_value="/remote/joint_states"),
        DeclareLaunchArgument(
            "local_joint_state_topic", default_value="/model/joint_states"),
        DeclareLaunchArgument(
            "torque_cmd_topic", default_value="/model/forward_torque_controller/commands"),
        DeclareLaunchArgument("p_gain", default_value="5.0"),
        DeclareLaunchArgument("d_gain", default_value="5.0"),
        DeclareLaunchArgument("torque_limit", default_value="10.0"),
        DeclareLaunchArgument("position_threshold", default_value="0.01"),
        DeclareLaunchArgument("timer_period_ms", default_value="1"),
        DeclareLaunchArgument(
            "ordered_joint_names",
            default_value="['fer_joint1','fer_joint2','fer_joint3','fer_joint4','fer_joint5','fer_joint6','fer_joint7']"),

        Node(
            package="delay_relay",
            executable="forward_torque_init_node",
            name="forward_torque_init_node",
            output="screen",
            parameters=[{
                "remote_joint_state_topic": LaunchConfiguration("remote_joint_state_topic"),
                "local_joint_state_topic": LaunchConfiguration("local_joint_state_topic"),
                "torque_cmd_topic": LaunchConfiguration("torque_cmd_topic"),
                "p_gain": LaunchConfiguration("p_gain"),
                "d_gain": LaunchConfiguration("d_gain"),
                "torque_limit": LaunchConfiguration("torque_limit"),
                "position_threshold": LaunchConfiguration("position_threshold"),
                "timer_period_ms": LaunchConfiguration("timer_period_ms"),
                "ordered_joint_names": LaunchConfiguration("ordered_joint_names"),
            }],
        ),
    ])
