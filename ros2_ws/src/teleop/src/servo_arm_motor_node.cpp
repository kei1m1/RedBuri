#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "redburi_msgs/msg/arm_motor.hpp"
#include "std_msgs/msg/float32.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ServoArmMotorNode : public rclcpp::Node
{
public:
  ServoArmMotorNode() : Node("servo_arm_motor_node")
  {
    max_joint_rpm_ = declare_parameter<double>("max_joint_rpm", 60.0);
    max_gripper_rpm_ = declare_parameter<double>("max_gripper_rpm", 30.0);
    servo_trajectory_topic_ = declare_parameter<std::string>(
      "servo_trajectory_topic", "/servo_node/joint_trajectory");
    arm_command_topic_ = declare_parameter<std::string>("arm_command_topic", "/arm_motor");
    gripper_command_topic_ = declare_parameter<std::string>(
      "gripper_command_topic", "/arm_gripper");

    trajectory_sub_ = create_subscription<trajectory_msgs::msg::JointTrajectory>(
      servo_trajectory_topic_,
      10,
      [this](const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
      {
        trajectoryCallback(msg);
      }
    );

    gripper_sub_ = create_subscription<std_msgs::msg::Float32>(
      gripper_command_topic_,
      10,
      [this](const std_msgs::msg::Float32::SharedPtr msg)
      {
        gripperCallback(msg);
      }
    );

    arm_pub_ = create_publisher<redburi_msgs::msg::ArmMotor>(arm_command_topic_, 10);
  }

private:
  static constexpr double RAD_PER_SEC_TO_RPM = 60.0 / (2.0 * M_PI);

  rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_sub_;
  rclcpp::Publisher<redburi_msgs::msg::ArmMotor>::SharedPtr arm_pub_;

  double max_joint_rpm_{};
  double max_gripper_rpm_{};
  double latest_gripper_command_{};
  std::string servo_trajectory_topic_{};
  std::string arm_command_topic_{};
  std::string gripper_command_topic_{};

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
    return std::clamp(rpm, -max_abs_rpm, max_abs_rpm);
  }

  void gripperCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    latest_gripper_command_ = std::clamp(static_cast<double>(msg->data), -1.0, 1.0);
  }

  void trajectoryCallback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
  {
    redburi_msgs::msg::ArmMotor arm{};
    arm.gripper_rpm = clampRpm(
      latest_gripper_command_ * max_gripper_rpm_, max_gripper_rpm_);

    if (msg->points.empty() || msg->joint_names.empty()) {
      arm_pub_->publish(arm);
      return;
    }

    const auto & point = msg->points.back();
    if (point.velocities.empty()) {
      arm_pub_->publish(arm);
      return;
    }

    std::array<double, 6> joint_rpm{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const size_t n = std::min(msg->joint_names.size(), point.velocities.size());

    for (size_t i = 0; i < n; ++i) {
      const int idx = jointIndexFromName(msg->joint_names[i]);
      if (idx < 0) {
        continue;
      }

      const double rpm = point.velocities[i] * RAD_PER_SEC_TO_RPM;
      joint_rpm[static_cast<size_t>(idx)] = clampRpm(rpm, max_joint_rpm_);
    }

    arm.joint_1_rpm = joint_rpm[0];
    arm.joint_2_rpm = joint_rpm[1];
    arm.joint_3_rpm = joint_rpm[2];
    arm.joint_4_rpm = joint_rpm[3];
    arm.joint_5_rpm = joint_rpm[4];
    arm.joint_6_rpm = joint_rpm[5];

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
