#ifndef STEERING_CONTROL_H
#define STEERING_CONTROL_H

#include <iostream>
#include <std_msgs/msg/float32.hpp>
#include "rclcpp/rclcpp.hpp"
#include <cmath>

using namespace std;
using namespace rclcpp;
using std::placeholders::_1;

class SteeringControl : public Node
{
private:
    Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
    Subscription<std_msgs::msg::Float32>::SharedPtr steering_angle_sub;
    Subscription<std_msgs::msg::Float32>::SharedPtr angular_velocity_sub;
    Subscription<std_msgs::msg::Float32>::SharedPtr setpoint_sub;
    TimerBase::SharedPtr timer;
    Time start_time;

    float kp;
    float ki;
    float kd;

    float steering_ref;  
    float steering_angle;
    float angular_velocity;

    float integral;
    float last_error;

    void timerCallback();
    void steeringAngleCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void angularVelocityCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void setpointCallback(const std_msgs::msg::Float32::SharedPtr msg);

public:
    SteeringControl(string name);
    ~SteeringControl();
};

#endif