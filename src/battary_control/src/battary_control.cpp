#include "battary_control.hpp"


BattaryControl::BattaryControl():Node("battary_node"){
    setVoltage();
    voltage_pub = this->create_publisher<float32Msg>("voltage", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(P2F(10)),
                                std::bind(&BattaryControl::currentCallback, this));
}

void BattaryControl::setVoltage(){
    this->max_voltage = cell_max_voltage * number_of_cell; 
    this->min_voltage = cell_min_voltage * number_of_cell; 
    this->voltage = this->max_voltage; 
}

void BattaryControl::linearDischarge(float current){
    float32Msg msg;
    float deltaTime = 1.0;

    float voltage_drop = current * internal_resistance;
    float effective_current = current - voltage_drop;

    voltage = voltage - effective_current * internal_resistance;

    if(voltage < 0.0){
        voltage = 0.0;
    }else if(voltage > max_voltage){
        voltage = max_voltage;
    }

    std::cout << "voltage : " << voltage  <<std::endl;
    msg.data = voltage;
    voltage_pub->publish(msg);
}

void BattaryControl::currentCallback(){
    linearDischarge(0.1);
}   


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BattaryControl>());
    rclcpp::shutdown();
    return 0;
}