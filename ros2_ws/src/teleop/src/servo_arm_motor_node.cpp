#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "std_msgs/msg/float32.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"

class ServoArmMotorNode : public rclcpp::Node
{
public:
  ServoArmMotorNode() : Node("servo_arm_motor_node")
  {
    max_joint_rpm_ = declare_parameter<double>("max_joint_rpm", 60.0);
    max_gripper_rpm_ = declare_parameter<double>("max_gripper_rpm", 30.0);

    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      "/servo_node/joint_trajectory",
      10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
      {
        trajectoryCallback(msg);
      }
    );

    gripper_sub_ = create_subscription<std_msgs::msg::Float32>(
      "/arm_gripper",
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        gripperCallback(msg);
      }
    );

    arm_pub_ = create_publisher<redburi_msgs::msg::ArmMotor>("/arm_motor", 10);
  }

private:
  static constexpr double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * M_PI);

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_sub_;
  rclcpp::Publisher<redburi_msgs::msg::ArmMotor>::SharedPtr arm_pub_;

  double max_joint_rpm_{};
  double max_gripper_rpm_{};
  double latest_gripper_command_{};
  bool has_last_trajectory_{false};
  rclcpp::Time last_trajectory_time_{0, 0, RCL_SYSTEM_TIME};
  std::array<double, 6> last_joint_position_{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  std::array<bool, 6> has_last_joint_position_{ false, false, false, false, false, false };

  static int jointIndexFromName(const std::string & name)
  {
    static const std::unordered_map<std::string, int> kMap{
      {"joint_1", 0},
      {"joint_2", 1},
      {"joint_3", 2},
      {"joint_4", 3},
      {"joint_5", 4},
      {"joint_6", 5},
    };

    const auto it = kMap.find(name);
    return (it == kMap.end()) ? -1 : it->second;
  }

  double clampRpm(double rpm, double max_abs_rpm) const
  {
    if(!std::isfinite(rpm))
    {
      return 0.0;
    }
    return std::clamp(rpm, -max_abs_rpm, max_abs_rpm);
  }

  void gripperCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_gripper_command_ = std::clamp(static_cast<double>(msg->data), -1.0, 1.0);
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    redburi_msgs::msg::ArmMotor arm{};

    if(msg->points.empty() || msg->joint_names.empty())
    {
      arm.gripper_rpm = clampRpm(latest_gripper_command_ * max_gripper_rpm_, max_gripper_rpm_);
      arm_pub_->publish(arm);
      return;
    }

    const auto & point = msg->points.back();
    std::array<double, 6> joint_rpm{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::array<double, 6> joint_rad_per_sec{ 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
    std::array<bool, 6> has_velocity{ false, false, false, false, false, false };
    const auto current_time = now();
    double dt = 0.0;

    if(has_last_trajectory_)
    {
      dt = (current_time - last_trajectory_time_).seconds();
    }

    const size_t velocity_count = std::min(msg->joint_names.size(), point.velocities.size());
    for(size_t i = 0; i < velocity_count; ++i)
    {
      const int idx = jointIndexFromName(msg->joint_names[i]);
      if(idx < 0)
      {
        continue;
      }

      const double velocity = point.velocities[i];
      if(!std::isfinite(velocity))
      {
        continue;
      }

      joint_rad_per_sec[static_cast<size_t>(idx)] = velocity;
      has_velocity[static_cast<size_t>(idx)] = true;
    }

    const size_t position_count = std::min(msg->joint_names.size(), point.positions.size());
    for(size_t i = 0; i < position_count; ++i)
    {
      const int idx = jointIndexFromName(msg->joint_names[i]);
      if(idx < 0)
      {
        continue;
      }

      const size_t joint_idx = static_cast<size_t>(idx);
      const double position = point.positions[i];
      if(!std::isfinite(position))
      {
        continue;
      }

      if((!has_velocity[joint_idx] || std::fabs(joint_rad_per_sec[joint_idx]) < 1e-9) &&
        has_last_trajectory_ && has_last_joint_position_[joint_idx] && dt > 1e-6)
      {
        joint_rad_per_sec[joint_idx] =
          (position - last_joint_position_[joint_idx]) / dt;
      }

      last_joint_position_[joint_idx] = position;
      has_last_joint_position_[joint_idx] = true;
    }

    for(size_t i = 0; i < joint_rpm.size(); ++i)
    {
      joint_rpm[i] = clampRpm(joint_rad_per_sec[i] * RAD_PER_SEC_TO_RPM, max_joint_rpm_);
    }

    last_trajectory_time_ = current_time;
    has_last_trajectory_ = true;

    arm.joint_1_rpm = joint_rpm[0];
    arm.joint_2_rpm = joint_rpm[1];
    arm.joint_3_rpm = joint_rpm[2];
    arm.joint_4_rpm = joint_rpm[3];
    arm.joint_5_rpm = joint_rpm[4];
    arm.joint_6_rpm = joint_rpm[5];
    arm.gripper_rpm = clampRpm(latest_gripper_command_ * max_gripper_rpm_, max_gripper_rpm_);

    arm_pub_->publish(arm);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ServoArmMotorNode>());
  rclcpp::shutdown();
  return 0;
}
