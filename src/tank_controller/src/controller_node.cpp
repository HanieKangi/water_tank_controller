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


class PIDController {
  private:
    double kp_;
    double ki_;
    double kd_;


    double min_output_;
    double max_output_;
    double prev_level_ = 0.0;
    double integral_;
    double prev_error_;
    bool first_run_;

  public:

    PIDController(double kp, double ki, double kd, double min_out, double max_out) :
      kp_(kp), ki_(ki), kd_(kd), 
      min_output_(min_out), max_output_(max_out),
      integral_(0.0), prev_error_(0.0), first_run_(true) {} 

      double compute(double setpoint,double measured_value, double dt) {

        double  error = setpoint - measured_value;

        double P = kp_ * error;

        integral_ += error * dt;
        double I = ki_ * integral_;

        double driative = 0.0;
        if (!first_run_) {
          driative = (error - prev_error_) / dt;
        }else {
          first_run_ = false;
        }
        double D = kd_ * driative;
        double output = P + I + D;
        double clamped_output = std::clamp(output , min_output_, max_output_);

        if (output != clamped_output && ki_ != 0.0) {
          integral_ -= error * dt;  
        }

        prev_error_ = error;


        return clamped_output;
      }

      void reset() {
        integral_ = 0.0;
        prev_error_ = 0.0;
        first_run_ = true;
      }

};


class TankController : public rclcpp::Node
{
  public:

    TankController()
    : Node("tank_controller"), 
      setpoint_(4.5),
      current_level_ (0.0), 
      pid_(32.0, 2, 15.0, 0.0, 100.0),
      last_time_(this->now())
    {
        RCLCPP_INFO(this->get_logger() , "Tank PID controller node started.");
        subscription_ = this->create_subscription<water_tank_interface::msg::TankOutput>(
        "water_level", 10 , std::bind(&TankController::water_level_callback, this, _1));

        publisher_ = this->create_publisher<water_tank_interface::msg::TankInput>("inlet_valve", 10);
        timer_ = this->create_wall_timer(
        100ms, std::bind(&TankController::timer_callback, this));
    }


  private:

    void water_level_callback(const water_tank_interface::msg::TankOutput & msg) 
    {
      current_level_ = msg.level;
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.level);
    }
    

    void timer_callback()
    {
      if (prev_level_ - current_level_ > 1.0) {
        pid_.reset();
        RCLCPP_INFO(this->get_logger(), "Simulator reset detected -> PID memory reset.");
      }
      rclcpp::Time current_time = this->now();
      double dt = (current_time - last_time_).seconds();
      if (dt <= 0.0) {
        return;
      }
      prev_level_ = current_level_;
      last_time_ = current_time;
      double valve_cmd = pid_.compute(setpoint_, current_level_, dt);
      auto command = water_tank_interface::msg::TankInput();
      command.inlet_valve= static_cast<float>(valve_cmd);
      RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", command.inlet_valve);
      publisher_->publish(command);
    }
    double prev_level_ = 0.0;
    double setpoint_;
    double current_level_;
    PIDController pid_;
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

