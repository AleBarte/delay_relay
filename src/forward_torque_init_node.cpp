#include "delay_relay/forward_torque_init_node.hpp"

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

ForwardTorqueInitNode::ForwardTorqueInitNode(const rclcpp::NodeOptions & options)
: Node("forward_torque_init_node", options)
{
  declareAndGetParameters();

  const std::string remote_js_topic =
    this->get_parameter("remote_joint_state_topic").as_string();
  const std::string local_js_topic =
    this->get_parameter("local_joint_state_topic").as_string();
  const std::string torque_cmd_topic =
    this->get_parameter("torque_cmd_topic").as_string();
  const int timer_ms =
    static_cast<int>(this->get_parameter("timer_period_ms").as_int());

  remote_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    remote_js_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ForwardTorqueInitNode::onRemoteJointState, this, std::placeholders::_1));

  local_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    local_js_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&ForwardTorqueInitNode::onLocalJointState, this, std::placeholders::_1));

  torque_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    torque_cmd_topic, rclcpp::SystemDefaultsQoS());

  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&ForwardTorqueInitNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(),
    "forward_torque_init_node started; waiting for '%s' and '%s'.",
    remote_js_topic.c_str(), local_js_topic.c_str());
}

void ForwardTorqueInitNode::declareAndGetParameters()
{
  this->declare_parameter<std::string>(
    "remote_joint_state_topic", "/remote/joint_states");
  this->declare_parameter<std::string>(
    "local_joint_state_topic", "/model/joint_states");
  this->declare_parameter<std::string>(
    "torque_cmd_topic", "/model/forward_torque_controller/commands");

  this->declare_parameter<double>("p_gain", 25.0);
  this->declare_parameter<double>("d_gain", 2.0);
  this->declare_parameter<double>("torque_limit", 10.0);
  this->declare_parameter<double>("position_threshold", 0.01);
  this->declare_parameter<int>("timer_period_ms", 10);
  this->declare_parameter<std::vector<std::string>>(
    "ordered_joint_names", defaultFrankaJointNames());

  p_gain_ = this->get_parameter("p_gain").as_double();
  d_gain_ = this->get_parameter("d_gain").as_double();
  torque_limit_ = this->get_parameter("torque_limit").as_double();
  position_threshold_ = this->get_parameter("position_threshold").as_double();
}

void ForwardTorqueInitNode::onRemoteJointState(
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

void ForwardTorqueInitNode::onLocalJointState(
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

void ForwardTorqueInitNode::publishTorqueCommand()
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();
  const std::size_t n = ordered_joint_names.size();

  if (
    remote_positions_.size() != n ||
    remote_velocities_.size() != n ||
    local_positions_.size() != n ||
    local_velocities_.size() != n)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping torque command: expected %zu ordered joints.", n);
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    const double position_error = remote_positions_[i] - local_positions_[i];
    const double velocity_error = remote_velocities_[i] - local_velocities_[i];
    const double raw_torque = p_gain_ * position_error + d_gain_ * velocity_error;
    msg.data[i] = std::clamp(raw_torque, -torque_limit_, torque_limit_);

    if (!std::isfinite(msg.data[i])) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skipping torque command: non-finite value computed for joint '%s'.",
        ordered_joint_names[i].c_str());
      return;
    }
  }

  torque_pub_->publish(msg);
}

void ForwardTorqueInitNode::publishZeroTorque()
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(ordered_joint_names.size(), 0.0);
  torque_pub_->publish(msg);
}

bool ForwardTorqueInitNode::isConverged() const
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
    if (std::abs(remote_positions_[i] - local_positions_[i]) > position_threshold_) {
      return false;
    }
  }
  return true;
}

void ForwardTorqueInitNode::onTimer()
{
  if (!remote_ready_ || !local_ready_) {
    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Waiting for joint state data ...");
    return;
  }

  if (isConverged()) {
    publishZeroTorque();
    torque_pub_.reset();
    timer_->cancel();
    RCLCPP_INFO(this->get_logger(),
      "Forward torque initialization complete. Torque publisher destroyed.");
    return;
  }

  publishTorqueCommand();
}

}  // namespace delay_relay
