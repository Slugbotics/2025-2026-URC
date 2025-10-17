#include "slugbot_package/SlugbotDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <cstdio>
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define HALF_DISTANCE_BETWEEN_WHEELS 0.045
#define WHEEL_RADIUS 0.025
#define WHEEL_COUNT 6

void set_position(WbDeviceTag *side, float value);
void set_velocity(WbDeviceTag *side, float value);

namespace slugbot_driver {
void SlugbotDriver::init(
    webots_ros2_driver::WebotsNode *node,
    std::unordered_map<std::string, std::string> &parameters) {

  left_motors[0] = wb_robot_get_device("FrontLeftWheel");
  left_motors[1] = wb_robot_get_device("MiddleLeftWheel");
  left_motors[2] = wb_robot_get_device("BackLeftWheel");
  left_side = left_motors;

  right_motors[0] = wb_robot_get_device("FrontRightWheel");
  right_motors[1] = wb_robot_get_device("MiddleRightWheel");
  right_motors[2] = wb_robot_get_device("BackRightWheel");
  right_side = right_motors;

  set_position(left_side, INFINITY);
  set_position(right_side, INFINITY);

  set_velocity(left_side, 0);
  set_velocity(right_side, 0);

  // wb_motor_set_position(left_motor, INFINITY);
  // wb_motor_set_velocity(left_motor, 0.0);

  // wb_motor_set_position(right_motor, INFINITY);
  // wb_motor_set_velocity(right_motor, 0.0);

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

  // wb_motor_set_velocity(left_motor, command_motor_left);
  // wb_motor_set_velocity(right_motor, command_motor_right);
  set_velocity(left_side, command_motor_left);
  set_velocity(right_side, command_motor_right);
}
} // namespace slugbot_driver

void set_position(WbDeviceTag *side, float value) {
  for (int i = 0; i < WHEEL_COUNT>>1; i++) {
    wb_motor_set_position(*side, value);
    side++;
  }
}

void set_velocity(WbDeviceTag *side, float value) {
  for (int i = 0; i < WHEEL_COUNT>>1; i++) {
    wb_motor_set_velocity(*side, value);
    side++;
  }
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(slugbot_driver::SlugbotDriver,
                       webots_ros2_driver::PluginInterface)