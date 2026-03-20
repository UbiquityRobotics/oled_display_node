cmake_minimum_required(VERSION 3.8)
project(oled_display_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DisplayOutput.msg"
  DEPENDENCIES std_msgs
)

# Include directories
include_directories(include)

# Executable
add_executable(oled_display_node src/oled_display_node.cpp)
ament_target_dependencies(oled_display_node
  rclcpp
  std_msgs
  sensor_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(oled_display_node "${cpp_typesupport_target}")

# Install
install(TARGETS oled_display_node
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/display_writer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
