#ifndef CONTROL_PKG__GAMEPAD_READER_H_
#define CONTROL_PKG__GAMEPAD_READER_H_

#include <chrono>
#include <optional>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/joy_feedback.hpp"
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

  // Modo bancada: segurar o botao de entrada/saida por bench_hold_time_s
  // altera o pedido publicado em /vida/safety/bench_mode_request.
  void update_bench_request(std::chrono::steady_clock::time_point now);

  // Vibra o controle quando o ESP32 confirma a troca de modo.
  void bench_mode_callback(const std_msgs::msg::Bool::SharedPtr msg);
  void update_rumble(std::chrono::steady_clock::time_point now);

  // Índices dos eixos utilizados na mensagem Joy.
  int left_stick_axis_;
  int r2_axis_;
  int l2_axis_;

  // Velocidade com R2 no fundo e fracao do L2 que aciona o freio.
  double max_speed_m_s_;
  double brake_threshold_;

  // Modo bancada: botoes, tempo segurando e estado.
  int bench_enter_button_;
  int bench_exit_button_;
  double bench_hold_time_s_;
  std::vector<int> buttons_;
  std::optional<std::chrono::steady_clock::time_point> enter_pressed_since_;
  std::optional<std::chrono::steady_clock::time_point> exit_pressed_since_;
  bool bench_requested_{false};
  std::optional<bool> bench_mode_active_;  // ultimo modo efetivo do ESP32

  // Padrao de vibracao em andamento: (duracao, intensidade) em sequencia.
  std::vector<std::pair<std::chrono::milliseconds, float>> rumble_pattern_;
  std::size_t rumble_step_{0};
  std::chrono::steady_clock::time_point rumble_step_started_;

  // Últimos valores recebidos dos controles.
  float left_stick_value_{0.0F};
  float r2_value_{1.0F};
  float l2_value_{1.0F};
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
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr bench_mode_request_publisher_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr bench_mode_subscription_;
  rclcpp::Publisher<sensor_msgs::msg::JoyFeedback>::SharedPtr rumble_publisher_;

  // Define a frequência de publicação das referências.
  rclcpp::TimerBase::SharedPtr reference_timer_;
};

#endif  // CONTROL_PKG__GAMEPAD_READER_H_
