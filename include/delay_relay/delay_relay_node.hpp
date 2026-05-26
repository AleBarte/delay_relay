#ifndef DELAY_RELAY__DELAY_RELAY_NODE_HPP_
#define DELAY_RELAY__DELAY_RELAY_NODE_HPP_

#include <deque>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace delay_relay
{

/**
 * @brief Relays a Float64MultiArray after a configurable delay.
 *
 * On startup the node enters INITIALIZING state:
 *   1. Reads one JointState from the remote manipulator.
 *   2. Publishes the joint positions as a Float64MultiArray command
 *      to the local manipulator.
 *   3. Waits until the local manipulator's joint error drops below
 *      `init_position_threshold` (rad), then switches to RELAYING.
 *
 * ROS 2 parameters
 * ----------------
 *  delay_sec               (double, default 1.0)   – Relay delay [s].
 *  input_topic             (string, default "~/input")
 *  output_topic            (string, default "~/output")
 *  remote_joint_state_topic (string, default "/remote/joint_states")
 *  local_joint_state_topic  (string, default "/local/joint_states")
 *  local_cmd_topic         (string, default "/local/joint_cmd")
 *  init_position_threshold (double, default 0.01)  – Convergence [rad].
 *  init_timeout_sec        (double, default 10.0)  – Max init wait [s].
 *  queue_size              (int,    default 100)
 *  timer_period_ms         (int,    default 10)
 */
class DelayRelayNode : public rclcpp::Node
{
public:
  explicit DelayRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- internal state machine -------------------------------------------
  enum class NodeState { INITIALIZING, RELAYING };

  // ---- callbacks --------------------------------------------------------
  void onMessage(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void onTimer();
  void onRemoteJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onLocalJointState(const sensor_msgs::msg::JointState::SharedPtr msg);

  // ---- initialization helpers -------------------------------------------
  void sendInitCommand();
  bool isLocalAtTarget() const;

  // ---- parameter helpers ------------------------------------------------
  void declareAndGetParameters();
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // ---- types ------------------------------------------------------------
  using StampedMsg =
    std::pair<rclcpp::Time, std_msgs::msg::Float64MultiArray::SharedPtr>;

  // ---- relay pub/sub ----------------------------------------------------
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr    pub_;

  // ---- initialisation pub/sub -------------------------------------------
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr remote_js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr local_js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr local_cmd_pub_;

  // ---- timer ------------------------------------------------------------
  rclcpp::TimerBase::SharedPtr drain_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  // ---- state ------------------------------------------------------------
  NodeState state_{NodeState::INITIALIZING};
  std::deque<StampedMsg> buffer_;

  // cached parameter values
  double delay_sec_{1.0};
  double init_threshold_{0.01};   ///< [rad] per-joint convergence tolerance
  double init_timeout_sec_{10.0};

  // init bookkeeping
  bool                     remote_js_received_{false};
  std::vector<double>      init_target_positions_;   ///< from remote JointState
  std::vector<double>      local_current_positions_; ///< from local  JointState
  rclcpp::Time             init_start_time_;
};

}  // namespace delay_relay

#endif  // DELAY_RELAY__DELAY_RELAY_NODE_HPP_
