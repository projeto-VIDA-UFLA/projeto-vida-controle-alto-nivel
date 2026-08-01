#include "vida_gamepad/gamepad_node.hpp"

#include <dirent.h>
#include <fcntl.h>
#include <linux/input.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <thread>
#include <vector>

namespace vida_gamepad
{
namespace
{
constexpr std::size_t kBitsPerLong = sizeof(unsigned long) * 8U;
constexpr std::size_t kEventBitArraySize = (EV_MAX / kBitsPerLong) + 1U;
constexpr std::size_t kAbsBitArraySize = (ABS_MAX / kBitsPerLong) + 1U;
constexpr std::size_t kKeyBitArraySize = (KEY_MAX / kBitsPerLong) + 1U;
}  // namespace

GamepadNode::GamepadNode(const rclcpp::NodeOptions & options)
: Node("vida_gamepad_node", options)
{
  device_name_ = declare_parameter<std::string>("device_name", "Wireless Controller");
  drive_axis_name_ = declare_parameter<std::string>("drive_axis", "ABS_RZ");
  steering_axis_name_ = declare_parameter<std::string>("steering_axis", "ABS_X");
  drive_max_ = declare_parameter<int>("drive_max", 254);
  steering_max_deg_ = declare_parameter<double>("steering_max_deg", 30.0);
  steering_deadzone_ = declare_parameter<double>("steering_deadzone", 0.08);
  invert_steering_ = declare_parameter<bool>("invert_steering", false);
  reconnect_interval_s_ = declare_parameter<double>("reconnect_interval", 2.0);
  const double publish_rate_hz = declare_parameter<double>("publish_rate", 50.0);

  drive_axis_code_ = axisCodeFromName(drive_axis_name_);
  steering_axis_code_ = axisCodeFromName(steering_axis_name_);

  drive_max_ = std::clamp(drive_max_, 0, 255);
  steering_max_deg_ = std::max(0.0, steering_max_deg_);
  steering_deadzone_ = std::clamp(steering_deadzone_, 0.0, 0.95);
  reconnect_interval_s_ = std::max(0.1, reconnect_interval_s_);

  drive_pub_ = create_publisher<std_msgs::msg::UInt8>(
    "/vida/control/drive/reference", rclcpp::QoS(10));
  steering_pub_ = create_publisher<std_msgs::msg::Float32>(
    "/vida/control/steering/reference", rclcpp::QoS(10));
  connected_pub_ = create_publisher<std_msgs::msg::Bool>(
    "/vida/safety/gamepad_connected", rclcpp::QoS(10).transient_local());

  const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_hz));
  publish_timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::milliseconds>(period),
    std::bind(&GamepadNode::publishReferences, this));

  setSafeState(false);
  {
    std_msgs::msg::Bool initial_connection_message;
    initial_connection_message.data = false;
    connected_pub_->publish(initial_connection_message);
  }

  RCLCPP_INFO(get_logger(), "Node VIDA Gamepad iniciado.");
  RCLCPP_INFO(
    get_logger(), "R2 (%s) -> /vida/control/drive/reference [0, %d]",
    drive_axis_name_.c_str(), drive_max_);
  RCLCPP_INFO(
    get_logger(), "Analógico esquerdo (%s) -> /vida/control/steering/reference [%.1f, %.1f] graus",
    steering_axis_name_.c_str(), -steering_max_deg_, steering_max_deg_);

  input_thread_ = std::thread(&GamepadNode::inputLoop, this);
}

GamepadNode::~GamepadNode()
{
  running_.store(false);
  if (input_thread_.joinable()) {
    input_thread_.join();
  }
  setSafeState(false);
}

void GamepadNode::inputLoop()
{
  while (running_.load() && rclcpp::ok()) {
    const std::string device_path = findGamepadDevice();

    if (device_path.empty()) {
      setSafeState(false);
      setConnected(false);
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_interval_s_));
      continue;
    }

    const int fd = open(device_path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
      RCLCPP_WARN(get_logger(), "Não foi possível abrir %s: %s", device_path.c_str(), std::strerror(errno));
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_interval_s_));
      continue;
    }

    if (!configureAxes(fd)) {
      close(fd);
      std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_interval_s_));
      continue;
    }

    setConnected(true);
    RCLCPP_INFO(get_logger(), "Controle conectado em %s", device_path.c_str());
    RCLCPP_INFO(
      get_logger(), "%s: mínimo=%d máximo=%d | %s: mínimo=%d máximo=%d",
      drive_axis_name_.c_str(), drive_raw_min_, drive_raw_max_,
      steering_axis_name_.c_str(), steering_raw_min_, steering_raw_max_);

    pollfd descriptor{};
    descriptor.fd = fd;
    descriptor.events = POLLIN;

    bool device_ok = true;
    while (running_.load() && rclcpp::ok() && device_ok) {
      const int poll_result = poll(&descriptor, 1, 500);

      if (poll_result < 0) {
        if (errno == EINTR) {
          continue;
        }
        device_ok = false;
        break;
      }

      if (poll_result == 0) {
        continue;
      }

      if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
        device_ok = false;
        break;
      }

      if ((descriptor.revents & POLLIN) != 0) {
        input_event event{};
        const ssize_t bytes = read(fd, &event, sizeof(event));

        if (bytes == static_cast<ssize_t>(sizeof(event))) {
          processInputEvent(event.type, event.code, event.value);
        } else if (bytes < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
          device_ok = false;
        }
      }
    }

    close(fd);
    const bool was_connected = connected_.load();
    setSafeState(was_connected);
    setConnected(false);
    std::this_thread::sleep_for(std::chrono::duration<double>(reconnect_interval_s_));
  }
}

std::string GamepadNode::findGamepadDevice() const
{
  DIR * directory = opendir("/dev/input");
  if (directory == nullptr) {
    return {};
  }

  std::vector<std::string> candidates;
  while (dirent * entry = readdir(directory)) {
    const std::string filename(entry->d_name);
    if (filename.rfind("event", 0) == 0) {
      candidates.emplace_back("/dev/input/" + filename);
    }
  }
  closedir(directory);
  std::sort(candidates.begin(), candidates.end());

  for (const auto & path : candidates) {
    const int fd = open(path.c_str(), O_RDONLY | O_NONBLOCK);
    if (fd < 0) {
      continue;
    }

    const bool matches = isMainGamepadDevice(fd, path);
    close(fd);

    if (matches) {
      return path;
    }
  }

  return {};
}

bool GamepadNode::isMainGamepadDevice(int fd, const std::string & path) const
{
  std::array<char, 256> name{};
  if (ioctl(fd, EVIOCGNAME(name.size()), name.data()) < 0) {
    return false;
  }

  if (device_name_ != std::string(name.data())) {
    return false;
  }

  std::array<unsigned long, kEventBitArraySize> event_bits{};
  std::array<unsigned long, kAbsBitArraySize> abs_bits{};
  std::array<unsigned long, kKeyBitArraySize> key_bits{};

  if (ioctl(fd, EVIOCGBIT(0, sizeof(event_bits)), event_bits.data()) < 0) {
    return false;
  }

  const bool has_abs = testBit(event_bits.data(), EV_ABS);
  const bool has_key = testBit(event_bits.data(), EV_KEY);
  if (!has_abs || !has_key) {
    return false;
  }

  if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits.data()) < 0 ||
    ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits.data()) < 0)
  {
    return false;
  }

  const bool has_drive_axis = testBit(abs_bits.data(), drive_axis_code_);
  const bool has_steering_axis = testBit(abs_bits.data(), steering_axis_code_);

  if (!has_drive_axis || !has_steering_axis) {
    RCLCPP_DEBUG(get_logger(), "Dispositivo ignorado (%s): eixos esperados ausentes", path.c_str());
    return false;
  }

  return true;
}

bool GamepadNode::configureAxes(int fd)
{
  input_absinfo drive_info{};
  input_absinfo steering_info{};

  if (ioctl(fd, EVIOCGABS(drive_axis_code_), &drive_info) < 0) {
    RCLCPP_ERROR(get_logger(), "Falha ao consultar o eixo %s", drive_axis_name_.c_str());
    return false;
  }

  if (ioctl(fd, EVIOCGABS(steering_axis_code_), &steering_info) < 0) {
    RCLCPP_ERROR(get_logger(), "Falha ao consultar o eixo %s", steering_axis_name_.c_str());
    return false;
  }

  drive_raw_min_ = drive_info.minimum;
  drive_raw_max_ = drive_info.maximum;
  steering_raw_min_ = steering_info.minimum;
  steering_raw_max_ = steering_info.maximum;

  drive_reference_.store(convertDriveValue(drive_info.value));
  steering_reference_.store(convertSteeringValue(steering_info.value));
  return true;
}

void GamepadNode::processInputEvent(unsigned short type, unsigned short code, int value)
{
  if (type != EV_ABS) {
    return;
  }

  if (code == drive_axis_code_) {
    drive_reference_.store(convertDriveValue(value));
  } else if (code == steering_axis_code_) {
    steering_reference_.store(convertSteeringValue(value));
  }
}

void GamepadNode::publishReferences()
{
  std_msgs::msg::UInt8 drive_message;
  std_msgs::msg::Float32 steering_message;

  if (connected_.load()) {
    drive_message.data = static_cast<uint8_t>(std::clamp(drive_reference_.load(), 0, 255));
    steering_message.data = steering_reference_.load();
  } else {
    drive_message.data = 0U;
    steering_message.data = 0.0F;
  }

  drive_pub_->publish(drive_message);
  steering_pub_->publish(steering_message);
}

void GamepadNode::setSafeState(bool log_disconnect)
{
  drive_reference_.store(0);
  steering_reference_.store(0.0F);

  if (log_disconnect) {
    RCLCPP_WARN(get_logger(), "Controle desconectado. Estado seguro: tração 0 e esterçamento 0°.");
  }
}

void GamepadNode::setConnected(bool connected)
{
  const bool previous = connected_.exchange(connected);
  if (previous == connected) {
    return;
  }

  std_msgs::msg::Bool message;
  message.data = connected;
  connected_pub_->publish(message);
}

int GamepadNode::axisCodeFromName(const std::string & axis_name)
{
  if (axis_name == "ABS_X") {return ABS_X;}
  if (axis_name == "ABS_Y") {return ABS_Y;}
  if (axis_name == "ABS_Z") {return ABS_Z;}
  if (axis_name == "ABS_RX") {return ABS_RX;}
  if (axis_name == "ABS_RY") {return ABS_RY;}
  if (axis_name == "ABS_RZ") {return ABS_RZ;}
  if (axis_name == "ABS_HAT0X") {return ABS_HAT0X;}
  if (axis_name == "ABS_HAT0Y") {return ABS_HAT0Y;}

  throw std::invalid_argument("Eixo evdev não suportado: " + axis_name);
}

bool GamepadNode::testBit(const unsigned long * bits, int bit)
{
  return (bits[bit / static_cast<int>(kBitsPerLong)] &
         (1UL << (bit % static_cast<int>(kBitsPerLong)))) != 0UL;
}

uint8_t GamepadNode::convertDriveValue(int raw_value) const
{
  if (drive_raw_max_ <= drive_raw_min_) {
    return 0U;
  }

  const double normalized = std::clamp(
    static_cast<double>(raw_value - drive_raw_min_) /
    static_cast<double>(drive_raw_max_ - drive_raw_min_),
    0.0, 1.0);

  const int output = static_cast<int>(std::lround(normalized * drive_max_));
  return static_cast<uint8_t>(std::clamp(output, 0, drive_max_));
}

float GamepadNode::convertSteeringValue(int raw_value) const
{
  if (steering_raw_max_ <= steering_raw_min_) {
    return 0.0F;
  }

  const double center = (static_cast<double>(steering_raw_min_) + steering_raw_max_) / 2.0;
  const double half_range = static_cast<double>(steering_raw_max_ - steering_raw_min_) / 2.0;
  double normalized = std::clamp((raw_value - center) / half_range, -1.0, 1.0);

  if (std::abs(normalized) < steering_deadzone_) {
    normalized = 0.0;
  } else {
    const double magnitude = (std::abs(normalized) - steering_deadzone_) /
      (1.0 - steering_deadzone_);
    normalized = std::copysign(magnitude, normalized);
  }

  if (invert_steering_) {
    normalized *= -1.0;
  }

  return static_cast<float>(std::clamp(
    normalized * steering_max_deg_, -steering_max_deg_, steering_max_deg_));
}

}  // namespace vida_gamepad
