#pragma once

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/transform_broadcaster.h"
#include "custom_nav_bot/action/go_to_point.hpp"

namespace custom_nav_bot
{
class NavServer : public rclcpp::Node
{
public:
  using GoToPoint = custom_nav_bot::action::GoToPoint;
  using GoalHandle = rclcpp_action::ServerGoalHandle<GoToPoint>;

  explicit NavServer(const rclcpp::NodeOptions & options);

private:
  // ROS 2 Interfaces
  rclcpp_action::Server<GoToPoint>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // TF2 Architecture
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string global_frame_;
  std::string robot_base_frame_;
  bool is_odom_active_{false};

  // Callbacks
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
  bool fetch_robot_pose(double & x, double & y, double & yaw);

  // Action Server Methods
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoToPoint::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(const std::shared_ptr<GoalHandle> goal_handle);
  void execute_navigation_state_machine(const std::shared_ptr<GoalHandle> goal_handle);
};
} // namespace custom_nav_bot