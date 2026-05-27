#ifndef DELAY_RELAY__FORWARD_TORQUE_DELAY_RELAY_NODE_HPP_
#define DELAY_RELAY__FORWARD_TORQUE_DELAY_RELAY_NODE_HPP_

#include <deque>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace delay_relay
{

class ForwardTorqueDelayRelayNode : public rclcpp::Node
{
public:
  explicit ForwardTorqueDelayRelayNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  enum class NodeState { INITIALIZING, RELAYING };

  struct StampedJointReference
  {
    rclcpp::Time stamp;
    std::vector<double> positions;
    std::vector<double> velocities;
  };

  void declareAndGetParameters();
  void onReferenceJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onRemoteJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onLocalJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTimer();
  void publishInitTorqueCommand();
  void publishRemoteTrackingTorque(const StampedJointReference & reference);
  void publishZeroInitTorque();
  bool isConverged() const;
  rcl_interfaces::msg::SetParametersResult onParameterChange(
    const std::vector<rclcpp::Parameter> & params);

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr reference_js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pub_;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr remote_js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr local_js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr init_torque_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;

  NodeState state_{NodeState::INITIALIZING};
  std::deque<StampedJointReference> buffer_;
  rclcpp::Time init_start_time_;

  std::vector<double> remote_positions_;
  std::vector<double> remote_velocities_;
  std::vector<double> local_positions_;
  std::vector<double> local_velocities_;
  bool remote_ready_{false};
  bool local_ready_{false};

  double delay_sec_{1.0};
  double p_gain_{25.0};
  double d_gain_{2.0};
  std::vector<double> p_gains_;
  std::vector<double> d_gains_;
  double torque_limit_{10.0};
  double init_threshold_{0.01};
  double init_timeout_sec_{10.0};
};

}  // namespace delay_relay

#endif  // DELAY_RELAY__FORWARD_TORQUE_DELAY_RELAY_NODE_HPP_
