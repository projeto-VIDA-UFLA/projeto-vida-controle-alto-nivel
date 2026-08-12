#ifndef CONTROL_PKG__GAMEPAD_READER_H_
#define CONTROL_PKG__GAMEPAD_READER_H_

#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/bool.hpp"
#include "vida_interfaces/msg/drive_reference.hpp"
#include "vida_interfaces/msg/steering_reference.hpp"

using namespace std;
using namespace rclcpp;

// Nó ROS 2 responsável por converter comandos do gamepad em referências
class GamepadReader : public rclcpp::Node
{
public:
  // Cria o nó com o nome informado e configura suas interfaces ROS 2.
  explicit GamepadReader(const std::string & node_name);

private:
  // Atualiza os valores dos controles a partir de uma mensagem do gamepad.
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg);

  // Publica as referências de direção e tração calculadas.
  void publish_references();

  // Índices dos eixos utilizados na mensagem Joy.
  int left_stick_axis_;
  int r2_axis_;

  // Últimos valores recebidos dos controles.
  float left_stick_value_{0.0F};
  float r2_value_{1.0F};
  bool enabled_{false};

  // Momento da ultima mensagem do gamepad, usado como watchdog.
  std::chrono::steady_clock::time_point last_joy_message_time_;
  bool joy_message_received_{false};

  // Recebe o estado atual do gamepad.
  Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

  // Enviam as referências para os controladores do veículo.
  rclcpp::Publisher<vida_interfaces::msg::SteeringReference>::SharedPtr steering_reference_publisher_;
  rclcpp::Publisher<vida_interfaces::msg::DriveReference>::SharedPtr drive_reference_publisher_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gamepad_enabled_publisher_;

  // Define a frequência de publicação das referências.
  rclcpp::TimerBase::SharedPtr reference_timer_;
};

#endif  // CONTROL_PKG__GAMEPAD_READER_H_
