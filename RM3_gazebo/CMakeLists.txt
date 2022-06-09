cmake_minimum_required(VERSION 3.5.0)
project(rm3_gazebo)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)


install(DIRECTORY
  launch
  config
  urdf
  worlds
  models
  DESTINATION share/${PROJECT_NAME}/
)

#install python modules
# ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS
    scripts/motor_to_body_vel.py
    scripts/whisker_state_publisher.py
    DESTINATION lib/${PROJECT_NAME}
)

add_executable(test_drive src/test_drive.cpp)
ament_target_dependencies(test_drive
  rclcpp
  geometry_msgs
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  ament_lint_auto_find_test_dependencies()
endif()

## Install C++ modules
install(
  TARGETS
    test_drive
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
