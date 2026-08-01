#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "vida_gamepad/gamepad_node.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<vida_gamepad::GamepadNode>());
  rclcpp::shutdown();
  return 0;
}
