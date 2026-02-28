#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/base_command.hpp"
#include "redburi_msgs/msg/base_motor.hpp"

class BaseCmdToMotor : public rclcpp::Node
{
public:
  BaseCmdToMotor() : Node("base_cmd_to_motor")
  {
    wheelbase_m_ = declare_parameter<double>("wheelbase_m");
    tread_m_ = declare_parameter<double>("tread_m");
    max_spin_rpm_ = declare_parameter<double>("max_spin_rpm");
    max_motor_rpm_ = declare_parameter<double>("max_motor_rpm");
    max_steer_deg_ = declare_parameter<double>("max_steer_deg");

    cmd_sub_ = create_subscription<redburi_msgs::msg::BaseCommand>(
      "/base_cmd",
      10,
      [this](redburi_msgs::msg::BaseCommand::SharedPtr msg)
      {
        baseCmdCallback(msg);
      }
    );
    motor_pub_ = create_publisher<redburi_msgs::msg::BaseMotor>("/base_motor", 10);
  }

private:
  double wheelbase_m_{};
  double tread_m_{};
  double max_spin_rpm_{};
  double max_motor_rpm_{};
  double max_steer_deg_{};
  rclcpp::Subscription<redburi_msgs::msg::BaseCommand>::SharedPtr cmd_sub_;
  rclcpp::Publisher<redburi_msgs::msg::BaseMotor>::SharedPtr motor_pub_;
  
  void baseCmdCallback(const redburi_msgs::msg::BaseCommand::SharedPtr msg)
  {
    redburi_msgs::msg::BaseMotor base;
    double drive{msg->drive};
    double steer{msg->steer};
    double spin{msg->spin};

    if(spin != 0)
    {
      base.steer_deg = 90.0;
      base.motor_f_rpm = max_spin_rpm_ * spin;
      base.motor_r_rpm = base.motor_f_rpm * (tread_m_ / 2.0) / wheelbase_m_;
      base.motor_l_rpm = -base.motor_r_rpm;
    }
    else if(drive != 0.0 || steer != 0.0)
    {
      base.steer_deg = max_steer_deg_ * steer;
      double steer_rad = std::fabs(base.steer_deg) * M_PI / 180.0; 
      base.motor_f_rpm = max_motor_rpm_ * drive;

      if(steer == 0)
      {
        base.motor_r_rpm = base.motor_f_rpm;
        base.motor_l_rpm = base.motor_f_rpm;  
      }
      else
      {
        double outer_rpm = base.motor_f_rpm * (wheelbase_m_ / std::tan(steer_rad) + tread_m_ / 2.0) / (wheelbase_m_ / std::sin(steer_rad));
        double inner_rpm = base.motor_f_rpm * (wheelbase_m_ / std::tan(steer_rad) - tread_m_ / 2.0) / (wheelbase_m_ / std::sin(steer_rad));

        if(steer > 0.0)
        {
          base.motor_r_rpm = outer_rpm;
          base.motor_l_rpm = inner_rpm;
        }
        else
        {
          base.motor_r_rpm = inner_rpm;
          base.motor_l_rpm = outer_rpm;
        }
      }
    }

    motor_pub_->publish(base);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<BaseCmdToMotor>());
  rclcpp::shutdown();
  return 0;
}
