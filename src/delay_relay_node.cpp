#include "delay_relay/delay_relay_node.hpp"

#include <chrono>
#include <stdexcept>
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

  // ---- resolve topic names -----------------------------------------------
  const std::string input_topic  = this->get_parameter("input_topic").as_string();
  const std::string output_topic = this->get_parameter("output_topic").as_string();
  const int         queue_size   = static_cast<int>(this->get_parameter("queue_size").as_int());
  const int         timer_ms     = static_cast<int>(this->get_parameter("timer_period_ms").as_int());

  // ---- QoS ---------------------------------------------------------------
  auto qos = rclcpp::QoS(rclcpp::KeepLast(queue_size));

  // ---- subscriber --------------------------------------------------------
  sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
    input_topic,
    qos,
    std::bind(&DelayRelayNode::onMessage, this, std::placeholders::_1));

  // ---- publisher ---------------------------------------------------------
  pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(output_topic, qos);

  // ---- drain timer -------------------------------------------------------
  drain_timer_ = this->create_wall_timer(
    std::chrono::milliseconds(timer_ms),
    std::bind(&DelayRelayNode::onTimer, this));

  // ---- live parameter updates --------------------------------------------
  param_cb_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DelayRelayNode::onParameterChange, this, std::placeholders::_1));

  RCLCPP_INFO(
    this->get_logger(),
    "delay_relay_node started — input: '%s'  output: '%s'  delay: %.3f s",
    input_topic.c_str(), output_topic.c_str(), delay_sec_);
}

// ---------------------------------------------------------------------------
// declareAndGetParameters
// ---------------------------------------------------------------------------
void DelayRelayNode::declareAndGetParameters()
{
  // delay_sec
  rcl_interfaces::msg::ParameterDescriptor delay_desc;
  delay_desc.description = "Message delay in seconds (>= 0)";
  delay_desc.floating_point_range.resize(1);
  delay_desc.floating_point_range[0].from_value = 0.0;
  delay_desc.floating_point_range[0].to_value   = 300.0;  // 5-min max – adjust if needed
  delay_desc.floating_point_range[0].step        = 0.0;   // continuous
  this->declare_parameter<double>("delay_sec", 1.0, delay_desc);

  // input / output topics
  this->declare_parameter<std::string>("input_topic",  "~/input");
  this->declare_parameter<std::string>("output_topic", "~/output");

  // queue / timer knobs
  this->declare_parameter<int>("queue_size",       100);
  this->declare_parameter<int>("timer_period_ms",   10);

  // cache the delay value
  delay_sec_ = this->get_parameter("delay_sec").as_double();
}

// ---------------------------------------------------------------------------
// onMessage  – called every time a new Float64MultiArray arrives
// ---------------------------------------------------------------------------
void DelayRelayNode::onMessage(
  const std_msgs::msg::Float64MultiArray::SharedPtr msg)
{
  buffer_.emplace_back(this->now(), msg);
}

// ---------------------------------------------------------------------------
// onTimer  – drains messages whose age has reached the requested delay
// ---------------------------------------------------------------------------
void DelayRelayNode::onTimer()
{
  const rclcpp::Time now = this->now();
  const rclcpp::Duration threshold =
    rclcpp::Duration::from_seconds(delay_sec_);

  while (!buffer_.empty()) {
    const auto & [stamp, msg] = buffer_.front();

    if ((now - stamp) >= threshold) {
      pub_->publish(*msg);
      buffer_.pop_front();
    } else {
      // Buffer is FIFO; no older message can be ready yet.
      break;
    }
  }
}

// ---------------------------------------------------------------------------
// onParameterChange  – allows runtime update of delay_sec
// ---------------------------------------------------------------------------
rcl_interfaces::msg::SetParametersResult DelayRelayNode::onParameterChange(
  const std::vector<rclcpp::Parameter> & params)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;

  for (const auto & p : params) {
    if (p.get_name() == "delay_sec") {
      const double new_delay = p.as_double();
      if (new_delay < 0.0) {
        result.successful = false;
        result.reason     = "delay_sec must be >= 0";
        return result;
      }
      delay_sec_ = new_delay;
      RCLCPP_INFO(this->get_logger(), "delay_sec updated to %.3f s", delay_sec_);
    }
  }

  return result;
}

}  // namespace delay_relay
