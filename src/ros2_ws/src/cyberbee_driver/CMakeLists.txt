cmake_minimum_required(VERSION 3.8)
project(cyberbee_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Set the include directories
include_directories(include/cyberbee_driver)

add_executable(cyberbee_driver_node src/driver_main.cpp src/data_receiver_ros.cpp src/serial_comm.cpp src/data_sender.cpp)

ament_target_dependencies(cyberbee_driver_node rclcpp std_msgs nav_msgs sensor_msgs)

install(TARGETS cyberbee_driver_node
  DESTINATION lib/${PROJECT_NAME})

# # Install the launch file
# install(DIRECTORY launch
#   DESTINATION share/${PROJECT_NAME})

ament_package()
