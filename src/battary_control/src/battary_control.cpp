#include "battary_control.hpp"


BattaryControl::BattaryControl():Node("battary_node"){
    setVoltage();
    info_pub = this->create_publisher<twistMsg>("info", 10);
    sbus_sub    = this->create_subscription<twistMsg>("sbus", 10, 
                                std::bind(&BattaryControl::sbusCallback, 
                                             this, std::placeholders::_1));
    arming_info_sub = this->create_subscription<int16Msg>("arming_info", 10, 
                                    std::bind(&BattaryControl::armingCallback, 
                                                this, std::placeholders::_1));

    timer_ = this->create_wall_timer(std::chrono::milliseconds(P2F(50)),
                                std::bind(&BattaryControl::currentCallback, this));
}

void BattaryControl::setVoltage(){
    this->max_voltage = cell_max_voltage * number_of_cells; 
    this->min_voltage = cell_min_voltage * number_of_cells; 
    this->voltage = this->max_voltage - 0.5;  
}

void BattaryControl::linearDischarge(float current){
    twistMsg msg;

    float voltage_drop = current * internal_resistance;
    float effective_current = current - voltage_drop;

    voltage = voltage - effective_current * internal_resistance;

    if(voltage < 0.0){
        voltage = 0.0;
    }else if(voltage > max_voltage){
        voltage = max_voltage;
    }

    std::cout << "voltage : " << voltage  <<std::endl;
    // msg.data = voltage;
    info_pub->publish(msg);
}

void BattaryControl::nanlinearDischarge(float current){
    twistMsg msg;
    float deltaTime = 1;

    float voltage_drop = current * internal_resistance;
    float effective_current = current - voltage_drop;

    float voltage_ratio = voltage / max_voltage ;
    
    float decay_factor = 1.0 - 1.0 / (1.0 + exp(-10 * (voltage_ratio)));

    if(decay_factor < 0.0002){
        decay_factor = 0.0002;
    } 

    float max_decay_factor = 1.0 - min_voltage / max_voltage;
    decay_factor *= max_decay_factor;

    voltage -= decay_factor * effective_current * internal_resistance;

    if (voltage < 0.0) {
        voltage = 0.0;
    } else if (voltage > max_voltage) {
        voltage = max_voltage;
    }
    if(voltage > 0.1){
        std::cout << "decay_factor: " << decay_factor <<std::endl;
        std::cout << "voltage : " << voltage << std::endl;
    }
    msg.linear.x = voltage;
    msg.linear.y = current * number_of_motors;
    info_pub->publish(msg);
}

void BattaryControl::nanlinearDischarge(float current, int id){

}

void BattaryControl::sbusCallback(const twistMsg::SharedPtr msg){
    float roll  = msg->angular.x;
    float pitch = msg->angular.y;
    float yaw   = msg->angular.z;
    float thrust = msg->linear.z;
    float current = thrust2current(thrust);
    if(is_arming == DISARM){
        current = 1.0;
    }
    std::cout << "thrust: " << thrust << " current: " << current << std::endl;
    nanlinearDischarge(current);
}

void BattaryControl::armingCallback(const int16Msg::SharedPtr msg){
    is_arming =  msg->data;
}

float BattaryControl::thrust2current(float thurst){
    return ((4.0  * pow(10, -5) * pow(thurst, 2)) - (0.09052 * thurst) +  52.0775);
}

void BattaryControl::currentCallback(){
    // linearDischarge(0.1);
    // nanlinearDischarge(40);
}   

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BattaryControl>());
    rclcpp::shutdown();
    return 0;
}