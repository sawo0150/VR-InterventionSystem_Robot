cmake_minimum_required(VERSION 3.8)
project(vris_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(vris_sensors REQUIRED)
find_package(vris_base_hw REQUIRED)
find_package(vris_slam REQUIRED)
find_package(vris_navigation REQUIRED)
# find_package(vris_vr_bridge REQUIRED)
find_package(vris_mode_manager REQUIRED)
find_package(vris_bt REQUIRED)
# find_package(vris_scenarios REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
