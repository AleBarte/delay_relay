#ifndef DELAY_RELAY__INIT_NODE_HPP_
#define DELAY_RELAY__INIT_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace delay_relay
{

/**
 * @brief Drives the local manipulator to the remote's joint positions
 *        using a proportional velocity controller.
 *
 * On every timer tick:
 *   vel_i = clamp(p_gain * (q_remote_i - q_local_i), -vel_limit, +vel_limit)
 *
 * When |error_i| < position_threshold for every joint:
 *   - A final zero-velocity command is published.
 *   - The velocity publisher is destroyed (reset to nullptr).
 *   - The timer is cancelled.
 *
 * ROS 2 parameters
 * ----------------
 *  remote_joint_state_topic  (string, default "/remote/joint_states")
 *  local_joint_state_topic   (string, default "/local/joint_states")
 *  velocity_cmd_topic        (string, default "/local/velocity_controller/commands")
 *  p_gain                    (double, default 2.0)   – proportional gain
 *  velocity_limit            (double, default 0.5)   – rad/s per-joint clamp
 *  position_threshold        (double, default 0.01)  – convergence [rad]
 *  timer_period_ms           (int,    default 10)
 */
class InitNode : public rclcpp::Node
{
public:
  explicit InitNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- callbacks --------------------------------------------------------
  void onRemoteJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onLocalJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTimer();

  // ---- helpers ----------------------------------------------------------
  void declareAndGetParameters();
  void publishVelocityCommand();
  void publishZeroVelocity();
  bool isConverged() const;

  // ---- ROS interfaces ---------------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr remote_js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr local_js_sub_;

  /// Destroyed (reset to nullptr) once convergence is reached.
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr vel_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  // ---- state ------------------------------------------------------------
  std::vector<double> remote_positions_;
  std::vector<double> local_positions_;
  bool remote_ready_{false};
  bool local_ready_{false};

  // ---- cached parameters ------------------------------------------------
  double p_gain_{2.0};
  double velocity_limit_{0.5};
  double position_threshold_{0.01};
};

}  // namespace delay_relay

#endif  // DELAY_RELAY__INIT_NODE_HPP_
