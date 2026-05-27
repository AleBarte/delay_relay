#include "delay_relay/forward_torque_delay_relay_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace
{

const std::vector<std::string> & defaultFrankaJointNames()
{
  static const std::vector<std::string> names = {
    "fer_joint1",
    "fer_joint2",
    "fer_joint3",
    "fer_joint4",
    "fer_joint5",
    "fer_joint6",
    "fer_joint7"};
  return names;
}

std::vector<double> repeatedGain(double gain, std::size_t size)
{
  return std::vector<double>(size, gain);
}

bool extractOrderedJointData(
  const sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & ordered_joint_names,
  std::vector<double> & ordered_positions,
  std::vector<double> & ordered_velocities,
  std::string & missing_joint)
{
  if (ordered_joint_names.empty()) {
    missing_joint = "<ordered_joint_names is empty>";
    return false;
  }

  ordered_positions.clear();
  ordered_velocities.clear();
  ordered_positions.reserve(ordered_joint_names.size());
  ordered_velocities.reserve(ordered_joint_names.size());

  for (const auto & joint_name : ordered_joint_names) {
    const auto name_it = std::find(msg.name.begin(), msg.name.end(), joint_name);
    if (name_it == msg.name.end()) {
      missing_joint = joint_name;
      return false;
    }

    const auto index = static_cast<std::size_t>(std::distance(msg.name.begin(), name_it));
    if (index >= msg.position.size()) {
      missing_joint = joint_name;
      return false;
    }

    ordered_positions.push_back(msg.position[index]);
    ordered_velocities.push_back(index < msg.velocity.size() ? msg.velocity[index] : 0.0);
  }

  return true;
}

}  // namespace

namespace delay_relay
{

ForwardTorqueDelayRelayNode::ForwardTorqueDelayRelayNode(
  const rclcpp::NodeOptions & options)
: Node("forward_torque_delay_relay_node", options)
{
  declareAndGetParameters();

  const std::string reference_js =
    this->get_parameter("reference_joint_state_topic").as_string();
  const std::string output_topic =
    this->get_parameter("output_topic").as_string();
  const std::string remote_js =
    this->get_parameter("remote_joint_state_topic").as_string();
  const std::string local_js =
    this->get_parameter("local_joint_state_topic").as_string();
  const std::string local_cmd =
    this->get_parameter("local_torque_cmd_topic").as_string();
  const int queue_size =
    static_cast<int>(this->get_parameter("queue_size").as_int());
  const int timer_ms =
    static_cast<int>(this->get_parameter("timer_period_ms").as_int());

  auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));

  reference_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    reference_js, rclcpp::SensorDataQoS(),
    std::bind(
      &ForwardTorqueDelayRelayNode::onReferenceJointState, this, std::placeholders::_1));

  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, qos);

  remote_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    remote_js, rclcpp::SensorDataQoS(),
    std::bind(&ForwardTorqueDelayRelayNode::onRemoteJointState, this, std::placeholders::_1));

  local_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    local_js, rclcpp::SensorDataQoS(),
    std::bind(&ForwardTorqueDelayRelayNode::onLocalJointState, this, std::placeholders::_1));

  init_torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    local_cmd, qos);

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&ForwardTorqueDelayRelayNode::onTimer, this));

  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&ForwardTorqueDelayRelayNode::onParameterChange, this, std::placeholders::_1));

  init_start_time_ = this->now();

  RCLCPP_INFO(this->get_logger(),
    "[INITIALIZING] Waiting for joint states on '%s' and '%s'. "
    "Relay reference topic is '%s'.",
    remote_js.c_str(), local_js.c_str(), reference_js.c_str());
}

void ForwardTorqueDelayRelayNode::declareAndGetParameters()
{
  auto float_range = [](double lo, double hi) {
    rcl_interfaces::msg::ParameterDescriptor d;
    d.floating_point_range.resize(1);
    d.floating_point_range[0].from_value = lo;
    d.floating_point_range[0].to_value = hi;
    d.floating_point_range[0].step = 0.0;
    return d;
  };

  this->declare_parameter<double>("delay_sec", 1.0, float_range(0.0, 300.0));
  this->declare_parameter<double>("p_gain", 25.0);
  this->declare_parameter<double>("d_gain", 2.0);
  this->declare_parameter<std::vector<double>>(
    "p_gains", repeatedGain(25.0, defaultFrankaJointNames().size()));
  this->declare_parameter<std::vector<double>>(
    "d_gains", repeatedGain(2.0, defaultFrankaJointNames().size()));
  this->declare_parameter<double>("torque_limit", 10.0);
  this->declare_parameter<double>("init_position_threshold", 0.01, float_range(0.0, 1.0));
  this->declare_parameter<double>("init_timeout_sec", 10.0, float_range(1.0, 120.0));

  this->declare_parameter<std::string>(
    "reference_joint_state_topic", "/model/joint_states");
  this->declare_parameter<std::string>(
    "output_topic", "/remote/forward_torque_controller/commands");
  this->declare_parameter<std::string>(
    "remote_joint_state_topic", "/remote/joint_states");
  this->declare_parameter<std::string>(
    "local_joint_state_topic", "/model/joint_states");
  this->declare_parameter<std::string>(
    "local_torque_cmd_topic", "/model/forward_torque_controller/commands");

  this->declare_parameter<int>("queue_size", 100);
  this->declare_parameter<int>("timer_period_ms", 10);
  this->declare_parameter<std::vector<std::string>>(
    "ordered_joint_names", defaultFrankaJointNames());

  delay_sec_ = this->get_parameter("delay_sec").as_double();
  p_gain_ = this->get_parameter("p_gain").as_double();
  d_gain_ = this->get_parameter("d_gain").as_double();
  p_gains_ = this->get_parameter("p_gains").as_double_array();
  d_gains_ = this->get_parameter("d_gains").as_double_array();
  torque_limit_ = this->get_parameter("torque_limit").as_double();
  init_threshold_ = this->get_parameter("init_position_threshold").as_double();
  init_timeout_sec_ = this->get_parameter("init_timeout_sec").as_double();

  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();
  if (p_gains_.size() != ordered_joint_names.size()) {
    RCLCPP_WARN(this->get_logger(),
      "p_gains has %zu entries, expected %zu. Falling back to scalar p_gain %.3f.",
      p_gains_.size(), ordered_joint_names.size(), p_gain_);
    p_gains_ = repeatedGain(p_gain_, ordered_joint_names.size());
  }
  if (d_gains_.size() != ordered_joint_names.size()) {
    RCLCPP_WARN(this->get_logger(),
      "d_gains has %zu entries, expected %zu. Falling back to scalar d_gain %.3f.",
      d_gains_.size(), ordered_joint_names.size(), d_gain_);
    d_gains_ = repeatedGain(d_gain_, ordered_joint_names.size());
  }
}

void ForwardTorqueDelayRelayNode::onRemoteJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> ordered_positions;
  std::vector<double> ordered_velocities;
  std::string missing_joint;
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  if (!extractOrderedJointData(
      *msg, ordered_joint_names, ordered_positions, ordered_velocities, missing_joint))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Remote JointState is missing configured joint '%s' or its position.",
      missing_joint.c_str());
    return;
  }

  remote_positions_ = std::move(ordered_positions);
  remote_velocities_ = std::move(ordered_velocities);
  remote_ready_ = true;
}

void ForwardTorqueDelayRelayNode::onLocalJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> ordered_positions;
  std::vector<double> ordered_velocities;
  std::string missing_joint;
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  if (!extractOrderedJointData(
      *msg, ordered_joint_names, ordered_positions, ordered_velocities, missing_joint))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Local JointState is missing configured joint '%s' or its position.",
      missing_joint.c_str());
    return;
  }

  local_positions_ = std::move(ordered_positions);
  local_velocities_ = std::move(ordered_velocities);
  local_ready_ = true;
}

void ForwardTorqueDelayRelayNode::publishInitTorqueCommand()
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();
  const std::size_t n = ordered_joint_names.size();

  if (
    remote_positions_.size() != n ||
    remote_velocities_.size() != n ||
    local_positions_.size() != n ||
    local_velocities_.size() != n ||
    p_gains_.size() != n ||
    d_gains_.size() != n)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping init torque command: expected %zu ordered joints.", n);
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    const double position_error = remote_positions_[i] - local_positions_[i];
    const double velocity_error = remote_velocities_[i] - local_velocities_[i];
    const double raw_torque = p_gains_[i] * position_error + d_gains_[i] * velocity_error;
    msg.data[i] = std::clamp(raw_torque, -torque_limit_, torque_limit_);

    if (!std::isfinite(msg.data[i])) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skipping init torque command: non-finite value computed for joint '%s'.",
        ordered_joint_names[i].c_str());
      return;
    }
  }

  init_torque_pub_->publish(msg);
}

void ForwardTorqueDelayRelayNode::publishRemoteTrackingTorque(
  const StampedJointReference & reference)
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();
  const std::size_t n = ordered_joint_names.size();

  if (
    reference.positions.size() != n ||
    reference.velocities.size() != n ||
    remote_positions_.size() != n ||
    remote_velocities_.size() != n ||
    p_gains_.size() != n ||
    d_gains_.size() != n)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping remote tracking torque: expected %zu ordered joints.", n);
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    const double position_error = reference.positions[i] - remote_positions_[i];
    const double velocity_error = reference.velocities[i] - remote_velocities_[i];
    const double raw_torque = p_gains_[i] * position_error + d_gains_[i] * velocity_error;
    msg.data[i] = std::clamp(raw_torque, -torque_limit_, torque_limit_);

    if (!std::isfinite(msg.data[i])) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skipping remote tracking torque: non-finite value computed for joint '%s'.",
        ordered_joint_names[i].c_str());
      return;
    }
  }

  pub_->publish(msg);
}

void ForwardTorqueDelayRelayNode::publishZeroInitTorque()
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(ordered_joint_names.size(), 0.0);
  init_torque_pub_->publish(msg);
}

bool ForwardTorqueDelayRelayNode::isConverged() const
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  if (
    remote_positions_.size() != ordered_joint_names.size() ||
    local_positions_.size() != ordered_joint_names.size())
  {
    return false;
  }

  for (std::size_t i = 0; i < remote_positions_.size(); ++i) {
    if (std::abs(remote_positions_[i] - local_positions_[i]) > init_threshold_) {
      return false;
    }
  }
  return true;
}

void ForwardTorqueDelayRelayNode::onTimer()
{
  if (state_ == NodeState::INITIALIZING) {
    if (!remote_ready_ || !local_ready_) {
      RCLCPP_INFO_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Waiting for joint state data ...");
      return;
    }

    publishInitTorqueCommand();

    if (isConverged()) {
      publishZeroInitTorque();
      state_ = NodeState::RELAYING;
      RCLCPP_INFO(this->get_logger(),
        "[RELAYING] Local arm reached init pose. Delay relay active (%.3f s).",
        delay_sec_);
      return;
    }

    const double elapsed = (this->now() - init_start_time_).seconds();
    if (elapsed > init_timeout_sec_) {
      publishZeroInitTorque();
      state_ = NodeState::RELAYING;
      RCLCPP_WARN(this->get_logger(),
        "[RELAYING] Init timeout (%.1f s) reached; starting relay anyway.",
        init_timeout_sec_);
    }
    return;
  }

  const rclcpp::Time now = this->now();
  const rclcpp::Duration threshold = rclcpp::Duration::from_seconds(delay_sec_);

  while (!buffer_.empty()) {
    const auto & reference = buffer_.front();
    if ((now - reference.stamp) >= threshold) {
      publishRemoteTrackingTorque(reference);
      buffer_.pop_front();
    } else {
      break;
    }
  }
}

void ForwardTorqueDelayRelayNode::onReferenceJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  if (state_ != NodeState::RELAYING) {
    return;
  }

  std::vector<double> ordered_positions;
  std::vector<double> ordered_velocities;
  std::string missing_joint;
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  if (!extractOrderedJointData(
      *msg, ordered_joint_names, ordered_positions, ordered_velocities, missing_joint))
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Reference JointState is missing configured joint '%s' or its position.",
      missing_joint.c_str());
    return;
  }

  buffer_.push_back({
    this->now(),
    std::move(ordered_positions),
    std::move(ordered_velocities)});
}

rcl_interfaces::msg::SetParametersResult
ForwardTorqueDelayRelayNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    const auto ordered_joint_names =
      this->get_parameter("ordered_joint_names").as_string_array();
    const auto expected_gain_count = ordered_joint_names.size();

    if (p.get_name() == "delay_sec") {
      if (p.as_double() < 0.0) {
        result.successful = false;
        result.reason = "delay_sec must be >= 0";
        return result;
      }
      delay_sec_ = p.as_double();
      RCLCPP_INFO(this->get_logger(), "delay_sec updated to %.3f s", delay_sec_);
    } else if (p.get_name() == "p_gain") {
      p_gain_ = p.as_double();
      p_gains_ = repeatedGain(p_gain_, expected_gain_count);
    } else if (p.get_name() == "d_gain") {
      d_gain_ = p.as_double();
      d_gains_ = repeatedGain(d_gain_, expected_gain_count);
    } else if (p.get_name() == "p_gains") {
      const auto new_gains = p.as_double_array();
      if (new_gains.size() != expected_gain_count) {
        result.successful = false;
        result.reason = "p_gains must have one entry per ordered joint";
        return result;
      }
      p_gains_ = new_gains;
    } else if (p.get_name() == "d_gains") {
      const auto new_gains = p.as_double_array();
      if (new_gains.size() != expected_gain_count) {
        result.successful = false;
        result.reason = "d_gains must have one entry per ordered joint";
        return result;
      }
      d_gains_ = new_gains;
    } else if (p.get_name() == "torque_limit") {
      torque_limit_ = p.as_double();
    } else if (p.get_name() == "init_position_threshold") {
      init_threshold_ = p.as_double();
    } else if (p.get_name() == "init_timeout_sec") {
      init_timeout_sec_ = p.as_double();
    }
  }
  return result;
}

}  // namespace delay_relay
