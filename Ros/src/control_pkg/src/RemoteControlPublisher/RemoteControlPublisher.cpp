#include "RemoteControlPublisher.h"

RemoteControl::RemoteControl(string name) : Node(name)
{
    udp = new UDP();
    steeringPublisher = this->create_publisher<std_msgs::msg::Int16>("/setpoint/steering", 10);
    speedPublisher = this->create_publisher<std_msgs::msg::Int16>("/setpoint/speed", 10);
    thread(&RemoteControl::publishControlsData, this).detach();
    RCLCPP_INFO(this->get_logger(), "Romote Control Has Been Started");
}

RemoteControl::~RemoteControl()
{
    delete udp;
}

void RemoteControl::publishControlsData()
{
    while (rclcpp::ok())
    {
        auto steeringSetpoint = std_msgs::msg::Int16();
        auto speedSetpoint = std_msgs::msg::Int16();

        // string data = udp->read();

        string data = udp->read();

        istringstream dataInt(data);
        int16_t steeringValue, speedValue;

        dataInt >> steeringValue >> speedValue;

        steeringSetpoint.data = steeringValue;
        speedSetpoint.data = speedValue;

        steeringPublisher->publish(steeringSetpoint);
        speedPublisher->publish(speedSetpoint);
    }
}

int main(int argc, char **argv)
{
    init(argc, argv);                                         // inicializado o ROS
    auto node = make_shared<RemoteControl>("remote_control"); // criação do nó usando ponteiro compartilhado e o nomeando
    spin(node);                                               // gira o nó (o mantem vivo até ctrl + c)
    shutdown();
    return 0;
}
