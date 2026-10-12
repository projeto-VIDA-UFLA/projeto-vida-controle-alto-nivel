#include <chrono>
#include <functional>
#include <algorithm>
#include <memory>

#include "gamepad_reader.h"

GamepadReader::GamepadReader(const std::string &node_name) : Node(node_name, "/gamepad")
{
  left_stick_axis_ = this->declare_parameter<int>("left_stick_axis", 0);
  r2_axis_ = this->declare_parameter<int>("r2_axis", 5);
  l2_axis_ = this->declare_parameter<int>("l2_axis", 2);

  joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>("joy", 10, std::bind(&GamepadReader::joy_callback, this, std::placeholders::_1));

  steering_reference_publisher_ = this->create_publisher<vida_interfaces::msg::SteeringReference>("/vida/control/steering/reference", 10);

  drive_reference_publisher_ = this->create_publisher<vida_interfaces::msg::DriveReference>("/vida/control/drive/reference", 10);

  gamepad_enabled_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/gamepad/enabled", 10);

  reference_timer_ = this->create_wall_timer(std::chrono::milliseconds(40), std::bind(&GamepadReader::publish_references, this));

  RCLCPP_INFO(this->get_logger(), "Lendo analogico esquerdo no eixo %d, R2 no eixo %d e L2 no eixo %d", left_stick_axis_, r2_axis_, l2_axis_);
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
  last_joy_message_time_ = std::chrono::steady_clock::now();  
  joy_message_received_ = true;
  enabled_ = true;
}

void GamepadReader::publish_references()
{
  constexpr auto joy_timeout = std::chrono::milliseconds(40);
  const auto now = std::chrono::steady_clock::now();

  if (!joy_message_received_ || now - last_joy_message_time_ > joy_timeout) {
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
  steering_reference.angle_deg = -15.0F * left_stick_value_;
  steering_reference.angular_velocity_deg_s = 0.0F;
  

  vida_interfaces::msg::DriveReference drive_reference;
  drive_reference.timestamp = timestamp;
  drive_reference.velocity_m_s = 127.5F * (1.0F - r2_value_);
  drive_reference.acceleration_m_s2 = 0.0F;
  // L2 solto (+1) = 0 (freio solto); L2 no fundo (-1) = 1 (freio total).
  drive_reference.brake_command = std::clamp(0.5F * (1.0F - l2_value_), 0.0F, 1.0F);

  steering_reference_publisher_->publish(steering_reference);
  drive_reference_publisher_->publish(drive_reference);
  gamepad_enabled_publisher_->publish(enabled_msg);
}

int main(int argc, char **argv)
{
  init(argc, argv);
  spin(std::make_shared<GamepadReader>("control"));
  shutdown();
  return 0;
}
