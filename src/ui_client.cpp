#include "custom_nav_bot/ui_client.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include <iostream>
#include <sstream>

namespace custom_nav_bot
{
UIClient::UIClient(const rclcpp::NodeOptions & options)
: Node("ui_client", options)
{
  client_ = rclcpp_action::create_client<GoToPoint>(this, "go_to_point");
  input_thread_ = std::thread([this]() { this->input_loop(); });
  
  RCLCPP_INFO(this->get_logger(), "--- Custom UI Client Started ---");
  RCLCPP_INFO(this->get_logger(), "Enter coordinates: X Y THETA");
  RCLCPP_INFO(this->get_logger(), "Or type 'cancel' to stop the robot.");
}

UIClient::~UIClient()
{
  running_ = false;
  if (input_thread_.joinable()) input_thread_.detach();
}

void UIClient::input_loop()
{
  // First prompt
  std::cout << "\n[Target Input] > ";

  while (rclcpp::ok() && running_) {
    std::string line;
    
    // Wait for the user to type something
    if (!std::getline(std::cin, line)) return;

    if (line == "cancel") {
      cancel_goal();
      continue;
    }

    std::stringstream ss(line);
    double x, y, theta;
    if (!(ss >> x >> y >> theta)) {
      std::cout << "[Error] Invalid format. Please use: X Y THETA\n";
      if (!goal_handle_) std::cout << "\n[Target Input] > ";
      continue;
    }
    
    send_goal(x, y, theta);
  }
}

void UIClient::send_goal(double x, double y, double theta)
{
  if (!client_->wait_for_action_server(std::chrono::seconds(3))) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available!");
    return;
  }

  auto goal_msg = GoToPoint::Goal();
  goal_msg.target_x = x;
  goal_msg.target_y = y;
  goal_msg.target_theta = theta;

  auto options = rclcpp_action::Client<GoToPoint>::SendGoalOptions();
  
  options.goal_response_callback = [this](const GoalHandle::SharedPtr & handle) {
    if (!handle) RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server.");
    else {
      goal_handle_ = handle;
      RCLCPP_INFO(this->get_logger(), "Goal accepted! Robot is moving.");
    }
  };

  options.feedback_callback = [this](
    GoalHandle::SharedPtr,
    const std::shared_ptr<const GoToPoint::Feedback> feedback) {
    // The \r rewrites the current line for a clean live-dashboard
    std::cout << "\r---> Distance remaining: " << feedback->distance_to_target << " meters   " << std::flush;
  };

  options.result_callback = [this](const GoalHandle::WrappedResult & result) {
    std::cout << "\n"; // Clear the feedback line
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        RCLCPP_INFO(this->get_logger(), "SUCCESS: %s", result.result->final_status.c_str());
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal aborted by server.");
        break;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_WARN(this->get_logger(), "Goal successfully canceled.");
        break;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code.");
        break;
    }
    goal_handle_.reset();
    
    // --- THE FIX: Explicitly reprint the prompt so the user knows it's ready ---
    std::cout << "\n[Target Input] > " << std::flush;
  };

  client_->async_send_goal(goal_msg, options);
}

void UIClient::cancel_goal()
{
  if (!goal_handle_) {
    RCLCPP_WARN(this->get_logger(), "No active goal to cancel.");
    // Reprint prompt if they randomly type cancel
    std::cout << "\n[Target Input] > " << std::flush;
    return;
  }
  client_->async_cancel_goal(goal_handle_);
  RCLCPP_INFO(this->get_logger(), "Sending cancel request...");
}
} // namespace custom_nav_bot

RCLCPP_COMPONENTS_REGISTER_NODE(custom_nav_bot::UIClient)