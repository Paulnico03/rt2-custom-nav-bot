#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "custom_nav_bot/action/go_to_point.hpp"
#include <thread>

namespace custom_nav_bot
{
class UIClient : public rclcpp::Node
{
public:
  using GoToPoint = custom_nav_bot::action::GoToPoint;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GoToPoint>;

  explicit UIClient(const rclcpp::NodeOptions & options);
  ~UIClient() override;

private:
  rclcpp_action::Client<GoToPoint>::SharedPtr client_;
  GoalHandle::SharedPtr goal_handle_;
  std::thread input_thread_;
  bool running_{true};

  void input_loop();
  void send_goal(double x, double y, double theta);
  void cancel_goal();
};
} // namespace custom_nav_bot