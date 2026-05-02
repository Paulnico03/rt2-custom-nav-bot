#include "rclcpp/rclcpp.hpp"
#include "custom_nav_bot/nav_server.hpp"
#include "custom_nav_bot/ui_client.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;

  // Create the executor
  rclcpp::executors::MultiThreadedExecutor executor;

  // Load the Server Component
  auto server_node = std::make_shared<custom_nav_bot::NavServer>(options);
  executor.add_node(server_node);

  // Load the Client UI Component
  auto client_node = std::make_shared<custom_nav_bot::UIClient>(options);
  executor.add_node(client_node);

  // Spin them both together!
  executor.spin();

  rclcpp::shutdown();
  return 0;
}