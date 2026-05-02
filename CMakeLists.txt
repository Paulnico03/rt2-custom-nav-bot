cmake_minimum_required(VERSION 3.8)
project(custom_nav_bot)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Register your custom action
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoToPoint.action"
)

ament_export_dependencies(rosidl_default_runtime)

include_directories(include)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# 1. The Navigation Server Library
add_library(nav_server_lib SHARED
  src/nav_server.cpp
)
ament_target_dependencies(nav_server_lib
  rclcpp rclcpp_action rclcpp_components geometry_msgs nav_msgs tf2 tf2_ros
)
target_link_libraries(nav_server_lib "${cpp_typesupport_target}")
rclcpp_components_register_nodes(nav_server_lib "custom_nav_bot::NavServer")

# 2. The User Interface Library
add_library(ui_client_lib SHARED
  src/ui_client.cpp
)
ament_target_dependencies(ui_client_lib
  rclcpp rclcpp_action rclcpp_components
)
target_link_libraries(ui_client_lib "${cpp_typesupport_target}")
rclcpp_components_register_nodes(ui_client_lib "custom_nav_bot::UIClient")

# 3. The Combined Container Executable
add_executable(run_robot_system
  src/main_container.cpp
)
ament_target_dependencies(run_robot_system
  rclcpp rclcpp_action geometry_msgs nav_msgs tf2 tf2_ros
)
target_link_libraries(run_robot_system
  nav_server_lib ui_client_lib "${cpp_typesupport_target}"
)

install(TARGETS
  nav_server_lib ui_client_lib run_robot_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_package()