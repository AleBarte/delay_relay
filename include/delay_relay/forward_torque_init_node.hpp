#ifndef DELAY_RELAY__FORWARD_TORQUE_INIT_NODE_HPP_
#define DELAY_RELAY__FORWARD_TORQUE_INIT_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

namespace delay_relay
{

class ForwardTorqueInitNode : public rclcpp::Node
{
public:
  explicit ForwardTorqueInitNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void declareAndGetParameters();
  void onRemoteJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onLocalJointState(const sensor_msgs::msg::JointState::SharedPtr msg);
  void onTimer();
  void publishTorqueCommand();
  void publishZeroTorque();
  bool isConverged() const;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr remote_js_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr local_js_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::vector<double> remote_positions_;
  std::vector<double> remote_velocities_;
  std::vector<double> local_positions_;
  std::vector<double> local_velocities_;
  bool remote_ready_{false};
  bool local_ready_{false};

  double p_gain_{25.0};
  double d_gain_{2.0};
  double torque_limit_{10.0};
  double position_threshold_{0.01};
};

}  // namespace delay_relay

#endif  // DELAY_RELAY__FORWARD_TORQUE_INIT_NODE_HPP_
