#ifndef VIDA_GAMEPAD__GAMEPAD_NODE_HPP_
#define VIDA_GAMEPAD__GAMEPAD_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/u_int8.hpp"

namespace vida_gamepad
{

class GamepadNode : public rclcpp::Node
{
public:
  explicit GamepadNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~GamepadNode() override;

private:
  void inputLoop();
  std::string findGamepadDevice() const;
  bool isMainGamepadDevice(int fd, const std::string & path) const;
  bool configureAxes(int fd);
  void processInputEvent(unsigned short type, unsigned short code, int value);
  void publishReferences();
  void setSafeState(bool log_disconnect);
  void setConnected(bool connected);

  static int axisCodeFromName(const std::string & axis_name);
  static bool testBit(const unsigned long * bits, int bit);

  uint8_t convertDriveValue(int raw_value) const;
  float convertSteeringValue(int raw_value) const;

  rclcpp::Publisher<std_msgs::msg::UInt8>::SharedPtr drive_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr connected_pub_;
  rclcpp::TimerBase::SharedPtr publish_timer_;

  std::string device_name_;
  std::string drive_axis_name_;
  std::string steering_axis_name_;

  int drive_axis_code_;
  int steering_axis_code_;
  int drive_max_;
  double steering_max_deg_;
  double steering_deadzone_;
  bool invert_steering_;
  double reconnect_interval_s_;

  std::atomic<bool> running_{true};
  std::atomic<bool> connected_{false};
  std::atomic<int> drive_reference_{0};
  std::atomic<float> steering_reference_{0.0F};

  int drive_raw_min_{0};
  int drive_raw_max_{255};
  int steering_raw_min_{0};
  int steering_raw_max_{255};

  std::thread input_thread_;
};

}  // namespace vida_gamepad

#endif  // VIDA_GAMEPAD__GAMEPAD_NODE_HPP_
