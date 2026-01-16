#include "SteeringControl.h"

SteeringControl::SteeringControl(string name) : Node(name)
{

    this->declare_parameter<double>("kp", 2.0);
    this->declare_parameter<double>("ki", 0.0);
    this->declare_parameter<double>("kd", 0.0);

    kp = this->get_parameter("kp").as_double();
    ki = this->get_parameter("ki").as_double();
    kd = this->get_parameter("kd").as_double();


    publisher = this->create_publisher<std_msgs::msg::Float32>("steer_cmd", 10);

    steering_angle_sub = this->create_subscription<std_msgs::msg::Float32>("/vehicle/steering_angle", 10, bind(&SteeringControl::steeringAngleCallback,this, _1));

    angular_velocity_sub = this->create_subscription<std_msgs::msg::Float32>("/vehicle/angular_velocity", 10, bind(&SteeringControl::angularVelocityCallback,this, _1));

    setpoint_sub = this->create_subscription<std_msgs::msg::Float32>("/vehicle/steering_ref", 10, bind(&SteeringControl::setpointCallback, this, _1));

    timer = this->create_wall_timer(chrono::milliseconds(20), bind(&SteeringControl::timerCallback, this));
    start_time = now();   

    steering_ref = 0.0f; // referência (graus)
    steering_angle = 0.0f;
    angular_velocity = 0.0f;

    integral = 0.0f;
    last_error = 0.0f;
}

SteeringControl::~SteeringControl()
{
}

void SteeringControl::steeringAngleCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    steering_angle = msg->data; // graus
}

/* Callback da velocidade angular */
void SteeringControl::angularVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    angular_velocity = msg->data;
}

void SteeringControl::setpointCallback(const std_msgs::msg::Float32::SharedPtr msg)
{
    steering_ref = msg->data; // graus
    RCLCPP_INFO(this->get_logger(), "Novo setpoint de ângulo de direção: %.2f graus", steering_ref);
}

void SteeringControl::timerCallback()
{

    auto msg = std_msgs::msg::Float32();

    /* Erro de posição */
    float error = steering_ref - steering_angle;

    /* Faixa morta de ±2 graus */
    if (std::abs(error) < 1.5f)
    {
        msg.data = 0.0f;
        publisher->publish(msg);
        return;
    }

    /* Integração (simples) */
    integral += error * 0.02f; // dt = 20 ms

    float derivative = (error - last_error) / 0.02f;
    last_error = error;

    /* PID
       - Termo D usando velocidade angular medida
    */
    float u = kp * error + ki * integral - kd * derivative;

    /* Saturador -1 a 1 */
    u = std::clamp(u, -1.0f, 1.0f);

    if (std::abs(u) < 0.4f)
    {
        u = 0.4f;
    }

    msg.data = u;
    publisher->publish(msg);
    
}

int main(int argc, char **argv)
{
    init(argc, argv);                                             // inicializado o ROS
    auto node = make_shared<SteeringControl>("steering_control"); // criação do nó usando ponteiro compartilhado e o nomeando
    spin(node);                                                   // gira o nó (o mantem vivo até ctrl + c)
    shutdown();
    return 0;
}
