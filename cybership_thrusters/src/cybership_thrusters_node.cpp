#include "cybership_thrusters/cybership_thrusters.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CybershipThruster>();
  node->initialize();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}