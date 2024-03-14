#include "battary_control.hpp"


BattaryControl::BattaryControl():Node("battary_node"){
    reset();
    Load();
    current_time =  xTaskGetTickCount();
    last_time =  xTaskGetTickCount();
}

void BattaryControl::reset(){
    // last_update_time_ = parent->GetWorld()->SimTime();
	charge_ = design_capacity_;
	voltage_ = constant_voltage_;
	temperature_ = design_temperature_;
	temp_set_ = design_temperature_;
	discharge_ = 0;
	current_drawn_ = 0;
	battery_empty_ = false;
	internal_cutt_off_ = false;
	model_initialised_ = true;
}

void BattaryControl::linearDischargeVoltageUpdate(){
    voltage_ = constant_voltage_ + lin_discharge_coeff_ 
                * (1 - discharge_ / design_capacity_) 
                - internal_resistance_ * current_lpf_;
}

void BattaryControl::nanlinearDischargeVoltageUpdate(){
    double t = CEL2KEL(temperature_);
    double t0 = CEL2KEL(design_temperature_);
    double E0T = constant_voltage_ + reversible_voltage_temp_ * (t - t0);
    double QT = design_capacity_ + capacity_temp_coeff_*(t-t0);  
	double KT = polarization_constant_ * exp(arrhenius_rate_polarization_ * (1 / t - 1 / t0));
    voltage_ = E0T - KT * QT/(QT + discharge_)
              * (current_lpf_ * (characteristic_time_ / 3600.0) - discharge_)
              + exponential_voltage_ * exp(-exponential_capacity_ * -discharge_);
}

void BattaryControl::currentCallback(const twistMsg msg){
    current_drawn_ = 0;
    current_drawn_ = 10.0; //msg gelicek
}

void BattaryControl::update(){
    current_time =  xTaskGetTickCount();
    uint64_t dt = (current_time - last_time);
    if(dt > MS2S(update_period_)){
        double n = dt / temp_lpf_tau_;
        temperature_ = temperature_ + n * (temp_set_ - temperature_);
        if (!battery_empty_) {
            double k = dt / lpf_tau_;
            current_lpf_ = current_lpf_ + k * (current_drawn_ - current_lpf_);
            if (!internal_cutt_off_) {
                if (use_nonlinear_model_){
                    nanlinearDischargeVoltageUpdate();
                }
                else {
                    linearDischargeVoltageUpdate();
                }
                charge_ = design_capacity_ + discharge_; // discharge is negative
                charge_memory_ = charge_;
            }
            if (voltage_<=cut_off_voltage_ && !internal_cutt_off_) {
                discharge_ = 0;
                voltage_ = 0;
                internal_cutt_off_ = true;
                charge_ = charge_memory_;
            }
        }

        if (!battery_empty_ && charge_<=0) {
            discharge_ = 0;
            current_lpf_ = 0;
            voltage_ = 0;
            battery_empty_ = true;
        }
        last_time =  xTaskGetTickCount();
    }
}

void BattaryControl::Load(){
    this->declare_parameter<int> ("technology", 2);
    this->declare_parameter<int> ("number_of_cells", 8);
    this->declare_parameter<int> ("num_of_consumers", 8);
    this->declare_parameter<double> ("update_rate", 10.0);
    this->declare_parameter<double> ("design_capacity", 4.0);
    this->declare_parameter<double> ("nominal_voltage", 24.0);
    this->declare_parameter<double> ("cut_off_voltage", 18.0);
    this->declare_parameter<double> ("full_charge_voltage", 24.2);
    this->declare_parameter<double> ("current_filter_tau", 1.0);
    this->declare_parameter<double> ("temperature_response_tau", 0.5);
    this->declare_parameter<double> ("internal_resistance", 0.05);
    this->declare_parameter<double> ("design_temperature", 25);

    this->declare_parameter<double> ("polarization_constant", 0.07);
    this->declare_parameter<double> ("exponential_voltage", 0.7);
    this->declare_parameter<double> ("exponential_capacity", 3.0);
    this->declare_parameter<double> ("characteristic_time", 200.0);
    this->declare_parameter<double> ("design_temperature", 25.0);
    this->declare_parameter<double> ("arrhenius_rate_polarization", 500);
    this->declare_parameter<double> ("reversible_voltage_temp", 0.05);
    this->declare_parameter<double> ("capacity_temp_coeff", 0.01);


    technology_  = this->get_parameter("technology").as_int();
    cell_count_  = this->get_parameter("number_of_cells").as_int();
    update_rate_ = this->get_parameter("update_rate").as_double();
    lpf_tau_     = this->get_parameter("current_filter_tau").as_double();
    num_of_consumers_ = this->get_parameter("num_of_consumers").as_int();
    design_capacity_  = this->get_parameter("design_capacity").as_double();
    nominal_voltage_  = this->get_parameter("nominal_voltage").as_double();
    cut_off_voltage_  = this->get_parameter("cut_off_voltage").as_double();
    constant_voltage_ = this->get_parameter("full_charge_voltage").as_double();
    temp_lpf_tau_     = this->get_parameter("temperature_response_tau").as_double();
    internal_resistance_ = this->get_parameter("internal_resistance").as_double();
    design_temperature_  = this->get_parameter("design_temperature").as_double();

   polarization_constant_ = this->get_parameter("polarization_constant").as_double();
   exponential_voltage_ = this->get_parameter("exponential_voltage").as_double();
   exponential_capacity_ = this->get_parameter("exponential_capacity").as_double();
   characteristic_time_ = this->get_parameter("characteristic_time").as_double();
   design_temperature_ = this->get_parameter("design_temperature").as_double();
   arrhenius_rate_polarization_ = this->get_parameter("arrhenius_rate_polarization").as_double();
   reversible_voltage_temp_ = this->get_parameter("reversible_voltage_temp").as_double();
   capacity_temp_coeff_ = this->get_parameter("capacity_temp_coeff").as_double();

   if(this->update_rate_ > 0.0){
        this->update_period_ = 1.0 / this->update_rate_;
   }else{
        this->update_period_ = 0.0;
   }
   temp_set_ = design_temperature_;
}

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    return 0;
}