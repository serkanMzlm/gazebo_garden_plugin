#ifndef __BATTARY_CONTROL_HPP__
#define __BATTARY_CONTROL_HPP__

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "battary_control_type.hpp"
#include "time_count.h"

using twistMsg = geometry_msgs::msg::Twist;


class BattaryControl: rclcpp::Node {
public:
    BattaryControl();
    void Load();
    void reset();
    void update();
    void setTemperature(float temp);
    void currentCallback(const twistMsg);
    void linearDischargeVoltageUpdate();
    void nanlinearDischargeVoltageUpdate();
    
private:
    bool publish_voltage_;
    int technology_;
    int num_of_consumers_;
    int cell_count_;
    double update_rate_;
    double update_period_;
    double design_capacity_;
    double current_drawn_;
    double nominal_voltage_;
    double constant_voltage_;
    double cut_off_voltage_;
    double internal_resistance_;
    double lpf_tau_;
    // linear model params
    double lin_discharge_coeff_;
    bool use_nonlinear_model_;
    // nonlinear model params
    double polarization_constant_; // polarization constant [V/Ah] or pol. resistance [Ohm]
    double exponential_voltage_;   // exponential voltage [V]
    double exponential_capacity_;  // exponential capacity [1/(Ah)]
    double characteristic_time_;   // characteristic time [s] for charge-dependence
    double design_temperature_;         // Design temperature where pol. const is unchanged and voltages are nominal.
    double arrhenius_rate_polarization_; // Arrhenius rate of polarization constant [K]
    double capacity_temp_coeff_;      // Temperature dependence of capacity [Ah/K]
    double reversible_voltage_temp_;  // Linear temperature dependant voltage shift [V/K]
    // internal variables
    bool model_initialised_;
    bool internal_cutt_off_;
    bool battery_empty_;
    double voltage_;
    double charge_;
    double charge_memory_;
    double current_;
    double discharge_;
    double capacity_;
    double temperature_;
    double temp_set_;
    double temp_lpf_tau_;
    double current_lpf_;

    uint64_t current_time;
    uint64_t last_time;
};


#endif