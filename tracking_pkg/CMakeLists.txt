cmake_minimum_required(VERSION 3.8)
project(tracking_pkg)

# Trova tutti i pacchetti necessari
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(tracking_msgs REQUIRED)


# Installa il pacchetto Python (chiama setup.py)
ament_python_install_package(${PROJECT_NAME})

# Funzioni finali
ament_export_dependencies(rosidl_default_runtime)

ament_package()
