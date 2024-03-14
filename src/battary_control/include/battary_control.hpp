#ifndef __BATTARY_CONTROL_HPP__
#define __BATTARY_CONTROL_HPP__

#include <iostream>
#include  <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"

#include "battary_control_type.hpp"

extern "C" {
#include "time_count.h"
}

using twistMsg = geometry_msgs::msg::Twist;
using float32Msg = std_msgs::msg::Float32;
using float32MultiArrayMsg = std_msgs::msg::Float32MultiArray;
using int16Msg = std_msgs::msg::Int16;

class BattaryControl: public rclcpp::Node {
public:
    BattaryControl();
    void setVoltage();
    void linearDischarge(float current);
    void nanlinearDischarge(float current);
    void nanlinearDischarge(float current, int id);

    void sbusCallback(const twistMsg::SharedPtr msg);
    void armingCallback(const int16Msg::SharedPtr msg);
    void currentCallback();
    float thrust2current(float thurst);
private:
    float temperature {25.0}; // [C]
    float capacity {5.2}; // [Ah]
    float nominal_voltage {3.6}; // [V]
    float internal_resistance {0.10}; // [ohm] 


    float cell_max_voltage {4.2}; // [V]
    float cell_min_voltage {2.5}; // [V]
    int number_of_cells {4};
    int number_of_motors {4};

    int is_arming {1000}; 

    float voltage;
    float max_voltage; // [V]
    float min_voltage; // [V]

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<twistMsg>::SharedPtr info_pub;
    rclcpp::Subscription<twistMsg>::SharedPtr sbus_sub;
    rclcpp::Subscription<int16Msg>::SharedPtr arming_info_sub;
};


#endif