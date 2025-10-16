#include "slugbot_package/SlugbotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025

namespace slugbot_driver {
void SlugbotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  right_motor = wb_robot_get_device("right wheel motor");
  left_motor = wb_robot_get_device("left wheel motor");

  wb_motor_set_position(left_motor, INFINITY);
  wb_motor_set_velocity(left_motor, 0.0);

  wb_motor_set_position(right_motor, INFINITY);
  wb_motor_set_velocity(right_motor, 0.0);

  cmd_vel_subscription_avoid_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_avoid", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg_avoid.linear = msg->linear;
        this->cmd_vel_msg_avoid.angular = msg->angular;
      }
  );

  cmd_vel_subscription_input_ = node->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel_input", rclcpp::SensorDataQoS().reliable(),
      [this](const geometry_msgs::msg::Twist::SharedPtr msg){
        this->cmd_vel_msg_input.linear = msg->linear;
        this->cmd_vel_msg_input.angular = msg->angular;
      }
  );
}

void SlugbotDriver::step() {
  auto forward_speed = cmd_vel_msg_avoid.linear.x + cmd_vel_msg_input.linear.x;
  auto angular_speed = cmd_vel_msg_avoid.angular.z + cmd_vel_msg_input.angular.z;

  auto command_motor_left =
      (forward_speed - angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;
  auto command_motor_right =
      (forward_speed + angular_speed * HALF_DISTANCE_BETWEEN_WHEELS) /
      WHEEL_RADIUS;

  wb_motor_set_velocity(left_motor, command_motor_left);
  wb_motor_set_velocity(right_motor, command_motor_right);
}
} // namespace slugbot_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(slugbot_driver::SlugbotDriver,
                       webots_ros2_driver::PluginInterface)