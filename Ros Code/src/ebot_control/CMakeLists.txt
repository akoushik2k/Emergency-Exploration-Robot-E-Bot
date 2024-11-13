cmake_minimum_required(VERSION 3.8)
project(ebot_control)

# Set default compiler options
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_control REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# Install directories for launch files, configuration, and scripts
install(DIRECTORY
  launch
  config
  scripts
  DESTINATION share/${PROJECT_NAME}/
)

# Install executable scripts
install(PROGRAMS
  scripts/ebot_control.py
  scripts/ebot_teleop.py
  scripts/ebot_nav.py
  DESTINATION lib/${PROJECT_NAME}
)

# Uncomment if Python package setup is needed for the scripts
# ament_python_install_package(${PROJECT_NAME})

# Install headers if using custom C++ headers in the include directory
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

# Enable testing if applicable
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
