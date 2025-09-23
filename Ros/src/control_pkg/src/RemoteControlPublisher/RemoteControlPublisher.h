#ifndef NUMBER_PUBLISH_H
#define NUMBER_PUBLISH_H

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int16.hpp"
#include "../../../../../Utils/UDP/UDP.h"
#include <sstream>

using namespace std;
using namespace rclcpp;

class RemoteControl : public Node
{
private:
    UDP *udp;
    Publisher<std_msgs::msg::Int16>::SharedPtr steeringPublisher;
    Publisher<std_msgs::msg::Int16>::SharedPtr speedPublisher;
    TimerBase::SharedPtr timer;

    void publishControlsData();

public:
    RemoteControl(string name);
    ~RemoteControl();
};

#endif