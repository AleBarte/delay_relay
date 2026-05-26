#include "delay_relay/init_node.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iterator>
#include <string>
#include <utility>
#include <vector>

namespace
{

const std::vector<std::string> & defaultOrderedJointNames()
{
  static const std::vector<std::string> names = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"};
  return names;
}

bool extractOrderedPositions(
  const sensor_msgs::msg::JointState & msg,
  const std::vector<std::string> & ordered_joint_names,
  std::vector<double> & ordered_positions,
  std::string & missing_joint)
{
  if (ordered_joint_names.empty()) {
    missing_joint = "<ordered_joint_names is empty>";
    return false;
  }

  ordered_positions.clear();
  ordered_positions.reserve(ordered_joint_names.size());

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
  }

  return true;
}

}  // namespace

namespace delay_relay
{

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
InitNode::InitNode(const rclcpp::NodeOptions & options)
: Node("init_node", options)
{
  declareAndGetParameters();

  const std::string remote_js_topic =
    this->get_parameter("remote_joint_state_topic").as_string();
  const std::string local_js_topic =
    this->get_parameter("local_joint_state_topic").as_string();
  const std::string vel_cmd_topic =
    this->get_parameter("velocity_cmd_topic").as_string();
  const int timer_ms =
    static_cast<int>(this->get_parameter("timer_period_ms").as_int());

  // ---- subscribers -------------------------------------------------------
  remote_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    remote_js_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&InitNode::onRemoteJointState, this, std::placeholders::_1));

  local_js_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
    local_js_topic,
    rclcpp::SensorDataQoS(),
    std::bind(&InitNode::onLocalJointState, this, std::placeholders::_1));

  // ---- velocity publisher ------------------------------------------------
  vel_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
    vel_cmd_topic, rclcpp::SystemDefaultsQoS());

  // ---- control timer -----------------------------------------------------
  timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&InitNode::onTimer, this));

  RCLCPP_INFO(this->get_logger(),
    "init_node started — waiting for joint state data on '%s' and '%s' ...",
    remote_js_topic.c_str(), local_js_topic.c_str());
}

// ---------------------------------------------------------------------------
// declareAndGetParameters
// ---------------------------------------------------------------------------
void InitNode::declareAndGetParameters()
{
  this->declare_parameter<std::string>(
    "remote_joint_state_topic", "/remote/joint_states");
  this->declare_parameter<std::string>(
    "local_joint_state_topic",  "/local/joint_states");
  this->declare_parameter<std::string>(
    "velocity_cmd_topic", "/local/velocity_controller/commands");

  this->declare_parameter<double>("p_gain",             2.0);
  this->declare_parameter<double>("velocity_limit",     0.5);
  this->declare_parameter<double>("position_threshold", 0.01);
  this->declare_parameter<int>   ("timer_period_ms",    10);
  this->declare_parameter<std::vector<std::string>>(
    "ordered_joint_names", defaultOrderedJointNames());

  p_gain_             = this->get_parameter("p_gain").as_double();
  velocity_limit_     = this->get_parameter("velocity_limit").as_double();
  position_threshold_ = this->get_parameter("position_threshold").as_double();
}

// ---------------------------------------------------------------------------
// Joint-state callbacks
// ---------------------------------------------------------------------------
void InitNode::onRemoteJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> ordered_positions;
  std::string missing_joint;
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  if (!extractOrderedPositions(*msg, ordered_joint_names, ordered_positions, missing_joint)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Remote JointState is missing configured joint '%s' or its position.",
      missing_joint.c_str());
    return;
  }

  remote_positions_ = std::move(ordered_positions);
  remote_ready_     = true;
}

void InitNode::onLocalJointState(
  const sensor_msgs::msg::JointState::SharedPtr msg)
{
  std::vector<double> ordered_positions;
  std::string missing_joint;
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  if (!extractOrderedPositions(*msg, ordered_joint_names, ordered_positions, missing_joint)) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Local JointState is missing configured joint '%s' or its position.",
      missing_joint.c_str());
    return;
  }

  local_positions_ = std::move(ordered_positions);
  local_ready_     = true;
}

// ---------------------------------------------------------------------------
// publishVelocityCommand  –  vel_i = clamp(p_gain * error_i, ±vel_limit)
// ---------------------------------------------------------------------------
void InitNode::publishVelocityCommand()
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();
  const std::size_t n = ordered_joint_names.size();

  if (remote_positions_.size() != n || local_positions_.size() != n) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "Skipping velocity command: expected %zu ordered joints, got remote=%zu local=%zu.",
      n, remote_positions_.size(), local_positions_.size());
    return;
  }

  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(n);

  for (std::size_t i = 0; i < n; ++i) {
    const double error   = remote_positions_[i] - local_positions_[i];
    const double raw_vel = p_gain_ * error;
    msg.data[i] = std::clamp(raw_vel, -velocity_limit_, velocity_limit_);

    if (!std::isfinite(msg.data[i])) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "Skipping velocity command: non-finite value computed for joint '%s'.",
        ordered_joint_names[i].c_str());
      return;
    }
  }

  vel_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// publishZeroVelocity
// ---------------------------------------------------------------------------
void InitNode::publishZeroVelocity()
{
  const auto ordered_joint_names =
    this->get_parameter("ordered_joint_names").as_string_array();

  std_msgs::msg::Float64MultiArray msg;
  msg.data.assign(ordered_joint_names.size(), 0.0);
  vel_pub_->publish(msg);
}

// ---------------------------------------------------------------------------
// isConverged  –  true when all joints are within threshold
// ---------------------------------------------------------------------------
bool InitNode::isConverged() const
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

// ---------------------------------------------------------------------------
// onTimer
// ---------------------------------------------------------------------------
void InitNode::onTimer()
{
  // Wait until both sides have provided at least one message
  if (!remote_ready_ || !local_ready_) {
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
      "Waiting for joint state data ...");
    return;
  }

  if (isConverged()) {
    publishZeroVelocity();
    vel_pub_.reset();   // destroy the publisher
    timer_->cancel();   // stop the timer — node is done
    RCLCPP_INFO(this->get_logger(),
      "Initialization complete. Velocity publisher destroyed.");
    return;
  }

  publishVelocityCommand();
}

}  // namespace delay_relay
