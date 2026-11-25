#include <memory>
#include <chrono>
#include <functional>
#include <string>
#include <iostream>
#include <algorithm>


#include "rclcpp/rclcpp.hpp"
#include "water_tank_interface/msg/tank_input.hpp"
#include "water_tank_interface/msg/tank_output.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;


class PIController {
  private:
    double kp_;
    double ki_;

    double min_output_;
    double max_output_;
    double integral_;
    
  public:

    PIController(double kp, double ki, double min_out, double max_out) :
      kp_(kp), ki_(ki), 
      min_output_(min_out), max_output_(max_out),
      integral_(0.0) {} 

      double compute(double setpoint,double measured_value, double dt) {

        double  error = setpoint - measured_value;

        double P = kp_ * error;
        
        double limit = 0.25 * setpoint;
        if(error < limit && error > -limit){
          integral_ += error * dt;
        }
        double I = ki_  * integral_;
        
        double output = P + I;
        double clamped_output = std::clamp(output , min_output_, max_output_);

        prev_error_ = error;

        return clamped_output;
      }
    
    void reset() {
      integral_ = 0.0;
    }

      
};


class TankController : public rclcpp::Node
{
  public:

    TankController()
    : Node("tank_controller"), 
      setpoint_(4.5),
      current_level_ (0.0), 
      pi_(37.0, 3.7, 0.0, 100.0),
      last_time_(this->now())
    {
        
        subscription_ = this->create_subscription<water_tank_interface::msg::TankOutput>(
        "water_level", 10 , std::bind(&TankController::water_level_callback, this, _1));

        publisher_ = this->create_publisher<water_tank_interface::msg::TankInput>("inlet_valve", 10);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&TankController::timer_callback, this));
    }


  private:

    void water_level_callback(const water_tank_interface::msg::TankOutput & msg) 
    {
      if (current_level_ - msg.level > 0.5) {
        pi_.reset();
        RCLCPP_WARN(
          this->get_logger(),
          "Simulation reset!"
        );
      }
      current_level_ = msg.level;
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.level);
    }
    

    void timer_callback()
    {
      rclcpp::Time current_time = this->now();
      double dt = (current_time - last_time_).seconds();
      if (dt <= 0.0) {
        return;
      }
     
      last_time_ = current_time;
      double valve_cmd = pi_.compute(setpoint_, current_level_, dt);
      auto command = water_tank_interface::msg::TankInput();
      command.inlet_valve= static_cast<float>(valve_cmd);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", command.inlet_valve);
      publisher_->publish(command);
    }
   
    double setpoint_;
    double current_level_;
    PIController pi_;
    rclcpp::Time last_time_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<water_tank_interface::msg::TankInput>::SharedPtr publisher_;
    rclcpp::Subscription<water_tank_interface::msg::TankOutput>::SharedPtr subscription_;

};


int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TankController>());
  rclcpp::shutdown();
  return 0;
}

