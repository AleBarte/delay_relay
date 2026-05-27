#include "rclcpp/rclcpp.hpp"
#include "delay_relay/forward_torque_init_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<delay_relay::ForwardTorqueInitNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
