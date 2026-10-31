#include <chrono>
#include <functional>
#include <algorithm>
#include <memory>
#include <string>

#include "gamepad_reader.h"

GamepadReader::GamepadReader(const std::string &node_name) : Node(node_name, "/gamepad")
{
  left_stick_axis_ = this->declare_parameter<int>("left_stick_axis", 0);
  r2_axis_ = this->declare_parameter<int>("r2_axis", 5);
  l2_axis_ = this->declare_parameter<int>("l2_axis", 2);
  max_speed_m_s_ = this->declare_parameter<double>("max_speed_m_s", 1.0);
  brake_threshold_ = this->declare_parameter<double>("brake_threshold", 0.1);
  bench_enter_button_ = this->declare_parameter<int>("bench_enter_button", 0);
  bench_exit_button_ = this->declare_parameter<int>("bench_exit_button", 1);
  bench_hold_time_s_ = this->declare_parameter<double>("bench_hold_time_s", 5.0);
  auto_enter_button_ = this->declare_parameter<int>("auto_enter_button", 2);
  auto_exit_button_ = this->declare_parameter<int>("auto_exit_button", 3);
  auto_hold_time_s_ = this->declare_parameter<double>("auto_hold_time_s", 5.0);
  maneuver_button_ = this->declare_parameter<int>("maneuver_button", 5);
  maneuver_hold_time_s_ = this->declare_parameter<double>("maneuver_hold_time_s", 3.0);
  joy_timeout_ = std::chrono::milliseconds(
    static_cast<int>(1000.0 * this->declare_parameter<double>("joy_timeout_s", 0.25)));
  // Sozinho, o gamepad comanda o ESP32 direto. Com a navegacao, publica em
  // /gamepad/* e o mode_manager escolhe entre ele e o Stanley.
  const auto steering_topic = this->declare_parameter<std::string>(
    "steering_reference_topic", "/vida/control/steering/reference");
  const auto drive_topic = this->declare_parameter<std::string>(
    "drive_reference_topic", "/vida/control/drive/reference");

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&GamepadReader::joy_callback, this, std::placeholders::_1));

  steering_reference_publisher_ = this->create_publisher<vida_interfaces::msg::SteeringReference>(steering_topic, 10);

  drive_reference_publisher_ = this->create_publisher<vida_interfaces::msg::DriveReference>(drive_topic, 10);

  gamepad_enabled_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/gamepad/enabled", 10);

  bench_mode_request_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/vida/safety/bench_mode_request", 10);

  bench_mode_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/vida/safety/bench_mode", 10, std::bind(&GamepadReader::bench_mode_callback, this, std::placeholders::_1));

  // Vibracao pelo joy_node (namespace /gamepad).
  rumble_publisher_ = this->create_publisher<sensor_msgs::msg::JoyFeedback>("joy/set_feedback", 10);

  auto_request_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/navigation/auto_request", 10);

  navigation_mode_subscription_ = this->create_subscription<vida_interfaces::msg::NavigationMode>("/navigation/mode", 10, std::bind(&GamepadReader::navigation_mode_callback, this, std::placeholders::_1));

  maneuver_subscription_ = this->create_subscription<std_msgs::msg::Bool>("/vida/safety/maneuver", 10, std::bind(&GamepadReader::maneuver_callback, this, std::placeholders::_1));

  reference_timer_ = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&GamepadReader::publish_references, this));

  RCLCPP_INFO(this->get_logger(), "Lendo analogico esquerdo no eixo %d, R2 no eixo %d e L2 no eixo %d", left_stick_axis_, r2_axis_, l2_axis_);
  RCLCPP_INFO(this->get_logger(), "Referencias em %s e %s", steering_topic.c_str(), drive_topic.c_str());
}

void GamepadReader::joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
{
  if (left_stick_axis_ < 0 || r2_axis_ < 0 || l2_axis_ < 0 ||
      static_cast<std::size_t>(left_stick_axis_) >= msg->axes.size() ||
      static_cast<std::size_t>(r2_axis_) >= msg->axes.size() ||
      static_cast<std::size_t>(l2_axis_) >= msg->axes.size()) {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Mensagem Joy nao possui os eixos configurados");
    return;
  }

  left_stick_value_ = std::clamp(msg->axes[left_stick_axis_], -1.0F, 1.0F);
  r2_value_ = std::clamp(msg->axes[r2_axis_], -1.0F, 1.0F);
  l2_value_ = std::clamp(msg->axes[l2_axis_], -1.0F, 1.0F);
  buttons_ = msg->buttons;
  last_joy_message_time_ = std::chrono::steady_clock::now();  
  joy_message_received_ = true;
  enabled_ = true;
}

void GamepadReader::publish_references()
{
  const auto now = std::chrono::steady_clock::now();

  if (!joy_message_received_ || now - last_joy_message_time_ > joy_timeout_) {
    left_stick_value_ = 0.0F;
    r2_value_ = 1.0F;
    // Sem gamepad o freio vai a total (L2 no fundo = -1).
    l2_value_ = -1.0F;
    enabled_ = false;
  }

  const rclcpp::Time timestamp = this->get_clock()->now();

  std_msgs::msg::Bool enabled_msg;
  enabled_msg.data = enabled_; 

  vida_interfaces::msg::SteeringReference steering_reference;
  steering_reference.timestamp = timestamp;
  // REP-103: analogico para a esquerda (+1 no joy) = angulo positivo.
  steering_reference.angle_deg = 15.0F * left_stick_value_;
  steering_reference.angular_velocity_deg_s = 0.0F;
  

  // Gatilhos: solto = +1, no fundo = -1. Fracao pressionada em [0, 1].
  const float r2_pressed = std::clamp(0.5F * (1.0F - r2_value_), 0.0F, 1.0F);
  const float l2_pressed = std::clamp(0.5F * (1.0F - l2_value_), 0.0F, 1.0F);
  const bool brake_requested = l2_pressed > brake_threshold_;
  update_maneuver_request(now, brake_requested || r2_pressed > brake_threshold_);

  // R2: velocidade de 0 a max_speed_m_s; o ESP32 controla tracao e freio.
  // L2: zera a velocidade e pede freio total (o ESP32 trata brake_command
  // acima de 0,05 como freio manual). Sem gamepad o L2 vai ao fundo.
  vida_interfaces::msg::DriveReference drive_reference;
  drive_reference.timestamp = timestamp;
  drive_reference.velocity_m_s =
    brake_requested ? 0.0F : static_cast<float>(max_speed_m_s_) * r2_pressed;
  drive_reference.acceleration_m_s2 = 0.0F;  // rampa padrao do ESP32
  drive_reference.brake_command = brake_requested ? 1.0F : 0.0F;
  if (maneuver_requested_) {
    // Freio solto a pedido (o ESP32 bloqueia a tracao nesse modo).
    drive_reference.velocity_m_s = 0.0F;
    drive_reference.brake_command = -1.0F;
  }

  steering_reference_publisher_->publish(steering_reference);
  drive_reference_publisher_->publish(drive_reference);
  gamepad_enabled_publisher_->publish(enabled_msg);

  update_bench_request(now);
  update_auto_request(now);
  update_rumble(now);
}

void GamepadReader::update_auto_request(std::chrono::steady_clock::time_point now)
{
  // buttons_ ja foi limpo por update_bench_request se o gamepad sumiu.
  const auto pressed = [this](int button) {
    return button >= 0 && static_cast<std::size_t>(button) < buttons_.size() &&
           buttons_[button] != 0;
  };
  const auto held = [&](bool is_pressed, std::optional<std::chrono::steady_clock::time_point> & since,
                        bool & sent) {
    if (!is_pressed) {
      since.reset();
      sent = false;
      return false;
    }
    if (!since) {
      since = now;
    }
    if (sent || std::chrono::duration<double>(now - *since).count() < auto_hold_time_s_) {
      return false;
    }
    sent = true;
    return true;
  };

  const bool enter = held(pressed(auto_enter_button_), auto_enter_pressed_since_, auto_enter_sent_);
  const bool exit = held(pressed(auto_exit_button_), auto_exit_pressed_since_, auto_exit_sent_);
  if (enter == exit) {
    return;
  }
  std_msgs::msg::Bool request;
  request.data = enter;
  auto_request_publisher_->publish(request);
  RCLCPP_WARN(this->get_logger(), enter ? "Modo AUTOMATICO pedido" : "Saida do modo automatico pedida");
}

void GamepadReader::navigation_mode_callback(const vida_interfaces::msg::NavigationMode::SharedPtr msg)
{
  using vida_interfaces::msg::NavigationMode;
  using std::chrono::milliseconds;
  // O primeiro valor recebido so registra o estado, sem vibrar.
  if (navigation_mode_) {
    const auto & last = *navigation_mode_;
    if (msg->refused_count != last.refused_count) {
      // Recusado: seis pulsos rapidos.
      start_rumble({{milliseconds(100), 1.0F}, {milliseconds(100), 0.0F},
                    {milliseconds(100), 1.0F}, {milliseconds(100), 0.0F},
                    {milliseconds(100), 1.0F}, {milliseconds(100), 0.0F},
                    {milliseconds(100), 1.0F}, {milliseconds(100), 0.0F},
                    {milliseconds(100), 1.0F}, {milliseconds(100), 0.0F},
                    {milliseconds(100), 1.0F}, {milliseconds(0), 0.0F}});
      RCLCPP_ERROR(this->get_logger(), "%s", msg->reason.c_str());
    } else if (msg->mode != last.mode) {
      if (msg->mode == NavigationMode::MODE_PREPARING) {
        // Pedido aceito: um pulso curto.
        start_rumble({{milliseconds(300), 1.0F}, {milliseconds(0), 0.0F}});
      } else if (msg->mode == NavigationMode::MODE_AUTOMATIC) {
        // Stanley no comando: dois pulsos longos.
        start_rumble({{milliseconds(500), 1.0F}, {milliseconds(300), 0.0F},
                      {milliseconds(500), 1.0F}, {milliseconds(0), 0.0F}});
      } else if (msg->mode == NavigationMode::MODE_STOPPING && msg->aborted) {
        // Aborto: vibracao continua longa.
        start_rumble({{milliseconds(2000), 1.0F}, {milliseconds(0), 0.0F}});
      } else if (msg->mode == NavigationMode::MODE_MANUAL) {
        // De volta ao manual: quatro pulsos curtos.
        start_rumble({{milliseconds(150), 1.0F}, {milliseconds(150), 0.0F},
                      {milliseconds(150), 1.0F}, {milliseconds(150), 0.0F},
                      {milliseconds(150), 1.0F}, {milliseconds(150), 0.0F},
                      {milliseconds(150), 1.0F}, {milliseconds(0), 0.0F}});
      }
      RCLCPP_WARN(this->get_logger(), "Navegacao: modo %u (%s)", msg->mode, msg->reason.c_str());
    }
  }
  navigation_mode_ = *msg;
}

void GamepadReader::update_maneuver_request(
  std::chrono::steady_clock::time_point now, bool r2_or_l2_pressed)
{
  const bool joy_fresh = joy_message_received_ && now - last_joy_message_time_ <= joy_timeout_;
  const bool pressed = joy_fresh && maneuver_button_ >= 0 &&
                       static_cast<std::size_t>(maneuver_button_) < buttons_.size() &&
                       buttons_[maneuver_button_] != 0;

  if (maneuver_requested_ && (r2_or_l2_pressed || !joy_fresh)) {
    maneuver_requested_ = false;
    RCLCPP_WARN(this->get_logger(), "Modo manobra cancelado (R2/L2 ou gamepad perdido)");
  }

  if (!pressed) {
    maneuver_pressed_since_.reset();
    maneuver_toggle_sent_ = false;
    return;
  }
  if (!maneuver_pressed_since_) {
    maneuver_pressed_since_ = now;
  }
  if (maneuver_toggle_sent_ ||
      std::chrono::duration<double>(now - *maneuver_pressed_since_).count() < maneuver_hold_time_s_) {
    return;
  }
  maneuver_toggle_sent_ = true;
  maneuver_requested_ = !maneuver_requested_;
  RCLCPP_WARN(this->get_logger(), maneuver_requested_
                                    ? "Modo manobra PEDIDO (freio solto; o ESP32 so entra parado)"
                                    : "Modo manobra desligado pelo gamepad");
}

void GamepadReader::maneuver_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  using std::chrono::milliseconds;
  // O primeiro valor recebido so registra o estado, sem vibrar.
  if (maneuver_active_ && *maneuver_active_ != msg->data) {
    if (msg->data) {
      // Freio solto: dois pulsos curtos.
      start_rumble({{milliseconds(200), 1.0F}, {milliseconds(150), 0.0F},
                    {milliseconds(200), 1.0F}, {milliseconds(0), 0.0F}});
      RCLCPP_WARN(this->get_logger(), "ESP32 em MODO MANOBRA: freio solto");
    } else {
      // Freio de volta: um pulso longo.
      start_rumble({{milliseconds(800), 1.0F}, {milliseconds(0), 0.0F}});
      RCLCPP_WARN(this->get_logger(), "ESP32 saiu do modo manobra: freio aplicado");
      maneuver_requested_ = false;
    }
  }
  maneuver_active_ = msg->data;
}

void GamepadReader::start_rumble(std::vector<std::pair<std::chrono::milliseconds, float>> pattern)
{
  rumble_pattern_ = std::move(pattern);
  rumble_step_ = 0;
  rumble_step_started_ = std::chrono::steady_clock::now();
}

void GamepadReader::update_bench_request(std::chrono::steady_clock::time_point now)
{
  // Sem gamepad por mais de 1 s o pedido volta ao modo normal.
  constexpr auto bench_joy_timeout = std::chrono::seconds(1);
  if (!joy_message_received_ || now - last_joy_message_time_ > bench_joy_timeout) {
    buttons_.clear();
    if (bench_requested_) {
      RCLCPP_WARN(this->get_logger(), "Gamepad perdido: pedido de modo bancada cancelado");
    }
    bench_requested_ = false;
  }

  const auto pressed = [this](int button) {
    return button >= 0 && static_cast<std::size_t>(button) < buttons_.size() &&
           buttons_[button] != 0;
  };
  const auto held = [&](bool is_pressed, std::optional<std::chrono::steady_clock::time_point> & since) {
    if (!is_pressed) {
      since.reset();
      return false;
    }
    if (!since) {
      since = now;
    }
    return std::chrono::duration<double>(now - *since).count() >= bench_hold_time_s_;
  };

  const bool enter_held = held(pressed(bench_enter_button_), enter_pressed_since_);
  const bool exit_held = held(pressed(bench_exit_button_), exit_pressed_since_);

  if (enter_held && !exit_held && !bench_requested_) {
    bench_requested_ = true;
    RCLCPP_WARN(this->get_logger(), "Modo bancada PEDIDO (o ESP32 so entra com o veiculo parado)");
  } else if (exit_held && bench_requested_) {
    bench_requested_ = false;
    RCLCPP_WARN(this->get_logger(), "Modo normal pedido");
  }

  std_msgs::msg::Bool request;
  request.data = bench_requested_;
  bench_mode_request_publisher_->publish(request);
}

void GamepadReader::bench_mode_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  // O primeiro valor recebido so registra o estado, sem vibrar.
  if (bench_mode_active_ && *bench_mode_active_ != msg->data) {
    using std::chrono::milliseconds;
    if (msg->data) {
      // Entrou: vibracao continua.
      rumble_pattern_ = {{milliseconds(1500), 1.0F}, {milliseconds(0), 0.0F}};
      RCLCPP_WARN(this->get_logger(), "ESP32 em MODO BANCADA");
    } else {
      // Saiu: tres pulsos.
      rumble_pattern_ = {
        {milliseconds(200), 1.0F}, {milliseconds(200), 0.0F},
        {milliseconds(200), 1.0F}, {milliseconds(200), 0.0F},
        {milliseconds(200), 1.0F}, {milliseconds(0), 0.0F}};
      RCLCPP_WARN(this->get_logger(), "ESP32 em modo normal");
    }
    rumble_step_ = 0;
    rumble_step_started_ = std::chrono::steady_clock::now();
  }
  bench_mode_active_ = msg->data;
}

void GamepadReader::update_rumble(std::chrono::steady_clock::time_point now)
{
  if (rumble_step_ >= rumble_pattern_.size()) {
    return;
  }

  // Avanca os passos cuja duracao ja terminou.
  while (rumble_step_ < rumble_pattern_.size() &&
         now - rumble_step_started_ >= rumble_pattern_[rumble_step_].first &&
         rumble_pattern_[rumble_step_].first.count() > 0) {
    rumble_step_started_ += rumble_pattern_[rumble_step_].first;
    ++rumble_step_;
  }

  sensor_msgs::msg::JoyFeedback feedback;
  feedback.type = sensor_msgs::msg::JoyFeedback::TYPE_RUMBLE;
  feedback.id = 0;
  feedback.intensity = rumble_step_ < rumble_pattern_.size()
                         ? rumble_pattern_[rumble_step_].second
                         : 0.0F;
  rumble_publisher_->publish(feedback);

  // Passo de duracao zero encerra o padrao (apos enviar a intensidade final).
  if (rumble_step_ < rumble_pattern_.size() &&
      rumble_pattern_[rumble_step_].first.count() == 0) {
    rumble_pattern_.clear();
    rumble_step_ = 0;
  }
}

int main(int argc, char **argv)
{
  init(argc, argv);
  spin(std::make_shared<GamepadReader>("control"));
  shutdown();
  return 0;
}
