cmake_minimum_required(VERSION 3.8)
project(ebot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ros2_control REQUIRED)

install(DIRECTORY
launch
config
scripts
DESTINATION share/${PROJECT_NAME}/
)

install(PROGRAMS
    scripts/ebot_control.py
    DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
    scripts/ebot_teleop.py
    DESTINATION lib/${PROJECT_NAME}
)
# install(PROGRAMS
#     scripts/ebot_waypoint.py
#     DESTINATION lib/${PROJECT_NAME}
# )
install(PROGRAMS
    scripts/ebot_nav.py
    DESTINATION lib/${PROJECT_NAME}
)
# ament_python_install_package(${PROJECT_NAME})

install(
    DIRECTORY include/ebot_control
    DESTINATION include
    # scripts/ebot_control.py

    )
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
