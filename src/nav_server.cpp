#include "custom_nav_bot/nav_server.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <thread>
#include <cmath>
#include <algorithm>

namespace custom_nav_bot
{
double wrap_angle(double angle) {
  while (angle > M_PI) angle -= 2.0 * M_PI;
  while (angle < -M_PI) angle += 2.0 * M_PI;
  return angle;
}

NavServer::NavServer(const rclcpp::NodeOptions & options)
: Node("nav_server", options)
{
  global_frame_ = this->declare_parameter<std::string>("global_frame", "odom");
  robot_base_frame_ = this->declare_parameter<std::string>("robot_frame", "base_link");

  cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
  odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
    "/odom", 10, std::bind(&NavServer::odom_callback, this, std::placeholders::_1));

  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  action_server_ = rclcpp_action::create_server<GoToPoint>(
    this, "go_to_point",
    std::bind(&NavServer::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
    std::bind(&NavServer::handle_cancel, this, std::placeholders::_1),
    std::bind(&NavServer::handle_accepted, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "--- Custom Navigation Server Online ---");
}

void NavServer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = msg->header.stamp;
  t.header.frame_id = global_frame_;
  t.child_frame_id = robot_base_frame_;
  t.transform.translation.x = msg->pose.pose.position.x;
  t.transform.translation.y = msg->pose.pose.position.y;
  t.transform.rotation = msg->pose.pose.orientation;
  
  tf_broadcaster_->sendTransform(t);
  is_odom_active_ = true;
}

bool NavServer::fetch_robot_pose(double & x, double & y, double & yaw)
{
  if (!is_odom_active_) return false;
  try {
    auto t = tf_buffer_->lookupTransform(global_frame_, robot_base_frame_, tf2::TimePointZero);
    x = t.transform.translation.x;
    y = t.transform.translation.y;
    double qx = t.transform.rotation.x; double qy = t.transform.rotation.y;
    double qz = t.transform.rotation.z; double qw = t.transform.rotation.w;
    yaw = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    return true;
  } catch (const tf2::TransformException & ex) {
    return false;
  }
}

rclcpp_action::GoalResponse NavServer::handle_goal(const rclcpp_action::GoalUUID &, std::shared_ptr<const GoToPoint::Goal>) {
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse NavServer::handle_cancel(const std::shared_ptr<GoalHandle>) {
  return rclcpp_action::CancelResponse::ACCEPT;
}

void NavServer::handle_accepted(const std::shared_ptr<GoalHandle> goal_handle) {
  std::thread{std::bind(&NavServer::execute_navigation_state_machine, this, goal_handle)}.detach();
}

void NavServer::execute_navigation_state_machine(const std::shared_ptr<GoalHandle> goal_handle)
{
  auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<GoToPoint::Feedback>();
  auto result = std::make_shared<GoToPoint::Result>();
  
  rclcpp::Rate loop_rate(20.0);
  int nav_state = 0; 

  while (rclcpp::ok()) {
    if (goal_handle->is_canceling()) {
      cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); 
      result->is_successful = false;
      result->final_status = "Canceled by user.";
      goal_handle->canceled(result);
      return;
    }

    double current_x, current_y, current_yaw;
    if (!fetch_robot_pose(current_x, current_y, current_yaw)) {
      loop_rate.sleep();
      continue;
    }

    double dx = goal->target_x - current_x;
    double dy = goal->target_y - current_y;
    double distance = std::hypot(dx, dy);
    double angle_to_target = std::atan2(dy, dx);
    
    double heading_error = wrap_angle(angle_to_target - current_yaw);
    double final_yaw_error = wrap_angle(goal->target_theta - current_yaw);

    feedback->distance_to_target = distance;
    goal_handle->publish_feedback(feedback);

    geometry_msgs::msg::Twist cmd;

    if (nav_state == 0) { 
      if (std::abs(heading_error) > 0.1) {
        cmd.angular.z = std::clamp(1.5 * heading_error, -1.0, 1.0);
      } else nav_state = 1; 
    } 
    else if (nav_state == 1) { 
      if (distance > 0.1) {
        cmd.linear.x = std::clamp(0.5 * distance, 0.0, 0.5);
        cmd.angular.z = std::clamp(1.0 * heading_error, -0.5, 0.5); 
      } else nav_state = 2; 
    } 
    else if (nav_state == 2) { 
      if (std::abs(final_yaw_error) > 0.05) {
        cmd.angular.z = std::clamp(1.5 * final_yaw_error, -1.0, 1.0);
      } else {
        cmd_vel_pub_->publish(geometry_msgs::msg::Twist()); 
        result->is_successful = true;
        result->final_status = "Target coordinates and heading reached perfectly!";
        goal_handle->succeed(result);
        return;
      }
    }

    cmd_vel_pub_->publish(cmd);
    loop_rate.sleep();
  }
}
} // namespace custom_nav_bot

RCLCPP_COMPONENTS_REGISTER_NODE(custom_nav_bot::NavServer)