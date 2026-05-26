#ifndef DELAY_RELAY__DELAY_RELAY_NODE_HPP_
#define DELAY_RELAY__DELAY_RELAY_NODE_HPP_

#include <deque>
#include <utility>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace delay_relay
{

/**
 * @brief Relays a Float64MultiArray topic after a configurable delay.
 *
 * Incoming messages are timestamped and stored in a FIFO queue.
 * A high-frequency timer drains the queue, publishing each message
 * once its age is >= the requested delay.
 *
 * ROS 2 parameters
 * ----------------
 *  delay_sec        (double, default 1.0)   – Delay in seconds (>= 0).
 *  input_topic      (string, default "~/input")  – Subscription topic.
 *  output_topic     (string, default "~/output") – Publisher topic.
 *  queue_size       (int,    default 100)   – QoS depth for sub/pub.
 *  timer_period_ms  (int,    default 10)    – Drain-timer period [ms].
 */
class DelayRelayNode : public rclcpp::Node
{
public:
  explicit DelayRelayNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // ---- callbacks --------------------------------------------------------
  void onMessage(const std_msgs::msg::Float64MultiArray::SharedPtr msg);
  void onTimer();

  // ---- parameter event handler -----------------------------------------
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  // ---- helpers ----------------------------------------------------------
  void declareAndGetParameters();

  // ---- data members -----------------------------------------------------
  using StampedMsg =
    std::pair<rclcpp::Time, std_msgs::msg::Float64MultiArray::SharedPtr>;

  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr drain_timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  std::deque<StampedMsg> buffer_;  ///< Pending messages, oldest first.

  // cached parameter values
  double delay_sec_{1.0};
};

}  // namespace delay_relay

#endif  // DELAY_RELAY__DELAY_RELAY_NODE_HPP_
