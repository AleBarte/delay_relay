#include "rclcpp/rclcpp.hpp"
#include "delay_relay/delay_relay_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<delay_relay::DelayRelayNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
