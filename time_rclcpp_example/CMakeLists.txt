cmake_minimum_required(VERSION 3.5)
project(time_rclcpp_example)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(time_example src/main.cpp)
ament_target_dependencies(time_example rclcpp std_msgs)

install(TARGETS time_example
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
