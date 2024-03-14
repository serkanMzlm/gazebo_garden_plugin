#ifndef __BATTARY_CONTROL_HPP__
#define __BATTARY_CONTROL_HPP__

#include <iostream>
#include  <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"

#include "battary_control_type.hpp"

extern "C" {
#include "time_count.h"
}

using twistMsg = geometry_msgs::msg::Twist;
using float32Msg = std_msgs::msg::Float32;

class BattaryControl: public rclcpp::Node {
public:
    BattaryControl();
    void setVoltage();
    void linearDischarge(float current);
    void currentCallback();
private:
    float temperature {25.0}; // [C]
    float capacity {5.2}; // [Ah]
    float nominal_voltage {3.6}; // [V]
    float internal_resistance {0.10}; // [ohm] 


    float cell_max_voltage {4.2}; // [V]
    float cell_min_voltage {2.5}; // [V]
    int number_of_cell {4};

    float voltage;
    float max_voltage; // [V]
    float min_voltage; // [V]

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<float32Msg>::SharedPtr voltage_pub;
};


#endif