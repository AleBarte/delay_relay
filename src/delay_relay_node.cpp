#include "delay_relay/delay_relay_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <string>

namespace delay_relay
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
DelayRelayNode::DelayRelayNode(const rclcpp::NodeOptions & options)
: Node("delay_relay_node", options)
{
  declareAndGetParameters();

  const std::string input_topic   = this->get_parameter("input_topic").as_string();
  const std::string output_topic  = this->get_parameter("output_topic").as_string();
  const std::string remote_js     = this->get_parameter("remote_joint_state_topic").as_string();
  const std::string local_js      = this->get_parameter("local_joint_state_topic").as_string();
  const std::string local_cmd     = this->get_parameter("local_cmd_topic").as_string();
  const int         queue_size    = static_cast<int>(this->get_parameter("queue_size").as_int());
  const int         timer_ms      = static_cast<int>(this->get_parameter("timer_period_ms").as_int());

  auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));

  // ---- relay pub/sub (active only in RELAYING state) --------------------
  sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    input_topic, qos,
    std::bind(&DelayRelayNode::onMessage, this, std::placeholders::_1));

  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, qos);

  // ---- initialisation pub/sub -------------------------------------------
  remote_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    remote_js, rclcpp::SensorDataQoS(),
    std::bind(&DelayRelayNode::onRemoteJointState, this, std::placeholders::_1));

  local_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    local_js, rclcpp::SensorDataQoS(),
    std::bind(&DelayRelayNode::onLocalJointState, this, std::placeholders::_1));

  local_cmd_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    local_cmd, qos);

  // ---- drain / init-check timer -----------------------------------------
  drain_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&DelayRelayNode::onTimer, this));

  // ---- live parameter updates -------------------------------------------
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DelayRelayNode::onParameterChange, this, std::placeholders::_1));

  init_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "[INITIALIZING] Waiting for remote joint state on '%s' ...", remote_js.c_str());
}

// ---------------------------------------------------------------------------
// declareAndGetParameters
// ---------------------------------------------------------------------------
void DelayRelayNode::declareAndGetParameters()
{
  auto float_range = [](double lo, double hi) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.floating_point_range.resize(1);
    d.floating_point_range[0].from_value = lo;
    d.floating_point_range[0].to_value   = hi;
    d.floating_point_range[0].step       = 0.0;
    return d;
  };

  this->declare_parameter<double>("delay_sec",               1.0,  float_range(0.0, 300.0));
  this->declare_parameter<double>("init_position_threshold", 0.01, float_range(0.0, 1.0));
  this->declare_parameter<double>("init_timeout_sec",        10.0, float_range(1.0, 120.0));

  this->declare_parameter<std::string>("input_topic",              "~/input");
  this->declare_parameter<std::string>("output_topic",             "~/output");
  this->declare_parameter<std::string>("remote_joint_state_topic", "/remote/joint_states");
  this->declare_parameter<std::string>("local_joint_state_topic",  "/local/joint_states");
  this->declare_parameter<std::string>("local_cmd_topic",          "/local/joint_cmd");

  this->declare_parameter<int>("queue_size",      100);
  this->declare_parameter<int>("timer_period_ms",  10);

  delay_sec_         = this->get_parameter("delay_sec").as_double();
  init_threshold_    = this->get_parameter("init_position_threshold").as_double();
  init_timeout_sec_  = this->get_parameter("init_timeout_sec").as_double();
}

// ---------------------------------------------------------------------------
// onRemoteJointState  – one-shot: capture target, send command, unsubscribe
// ---------------------------------------------------------------------------
void DelayRelayNode::onRemoteJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (remote_js_received_) {
    return;  // already captured — ignore further messages
  }

  if (msg->position.empty()) {
    RCLCPP_WARN(this->get_logger(), "Remote JointState has no position data — ignoring.");
    return;
  }

  remote_js_received_    = true;
  init_target_positions_ = msg->position;

  RCLCPP_INFO(this->get_logger(),
    "[INITIALIZING] Remote joint state captured (%zu joints). Sending init command ...",
    init_target_positions_.size());

  sendInitCommand();
}

// ---------------------------------------------------------------------------
// onLocalJointState  – track local arm position during init
// ---------------------------------------------------------------------------
void DelayRelayNode::onLocalJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (!msg->position.empty()) {
    local_current_positions_ = msg->position;
  }
}

// ---------------------------------------------------------------------------
// sendInitCommand  – publish target positions to local arm
// ---------------------------------------------------------------------------
void DelayRelayNode::sendInitCommand()
{
  std_msgs::msg::Float64MultiArray cmd;
  cmd.data = init_target_positions_;
  local_cmd_pub_->publish(cmd);
}

// ---------------------------------------------------------------------------
// isLocalAtTarget  – check per-joint error against threshold
// ---------------------------------------------------------------------------
bool DelayRelayNode::isLocalAtTarget() const
{
  if (local_current_positions_.size() != init_target_positions_.size()) {
    return false;  // sizes not yet consistent
  }

  for (std::size_t i = 0; i < init_target_positions_.size(); ++i) {
    if (std::abs(local_current_positions_[i] - init_target_positions_[i]) > init_threshold_) {
      return false;
    }
  }
  return true;
}

// ---------------------------------------------------------------------------
// onTimer  – handles both INITIALIZING and RELAYING phases
// ---------------------------------------------------------------------------
void DelayRelayNode::onTimer()
{
  // ---- INITIALIZING phase -----------------------------------------------
  if (state_ == NodeState::INITIALIZING) {

    // Keep re-sending the command at timer rate until confirmed or timeout
    if (remote_js_received_) {
      sendInitCommand();  // idempotent: safe to repeat until convergence

      if (isLocalAtTarget()) {
        state_ = NodeState::RELAYING;
        RCLCPP_INFO(this->get_logger(),
          "[RELAYING] Local arm reached init pose. Delay relay active (%.3f s).",
          delay_sec_);
        return;
      }

      // Timeout guard
      const double elapsed = (this->now() - init_start_time_).seconds();
      if (elapsed > init_timeout_sec_) {
        RCLCPP_WARN(this->get_logger(),
          "[RELAYING] Init timeout (%.1f s) reached — starting relay anyway. "
          "Check local arm controller.", init_timeout_sec_);
        state_ = NodeState::RELAYING;
      }
    }
    return;  // do NOT relay during init
  }

  // ---- RELAYING phase ---------------------------------------------------
  const rclcpp::Time     now       = this->now();
  const rclcpp::Duration threshold = rclcpp::Duration::from_seconds(delay_sec_);

  while (!buffer_.empty()) {
    const auto & [stamp, msg] = buffer_.front();
    if ((now - stamp) >= threshold) {
      pub_->publish(*msg);
      buffer_.pop_front();
    } else {
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// onMessage  – buffer incoming messages (only meaningful in RELAYING state)
// ---------------------------------------------------------------------------
void DelayRelayNode::onMessage(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  if (state_ == NodeState::RELAYING) {
    buffer_.emplace_back(this->now(), msg);
  }
  // Drop messages received during INITIALIZING to avoid stale data burst
}

// ---------------------------------------------------------------------------
// onParameterChange
// ---------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult DelayRelayNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "delay_sec") {
      if (p.as_double() < 0.0) {
        result.successful = false;
        result.reason = "delay_sec must be >= 0";
        return result;
      }
      delay_sec_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "delay_sec updated to %.3f s", delay_sec_);
    } else if (p.get_name() == "init_position_threshold") {
      init_threshold_ = p.as_double();
    } else if (p.get_name() == "init_timeout_sec") {
      init_timeout_sec_ = p.as_double();
    }
  }
  return result;
}

}  // namespace delay_relay
