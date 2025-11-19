cmake_minimum_required(VERSION 3.8)
project(tracking_pkg)

find_package(ament_cmake REQUIRED)

find_package(ament_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tracking_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

ament_export_dependencies(rosidl_default_runtime tracking_msgs)
ament_package()