cmake_minimum_required(VERSION 3.8)
project(launch)

find_package(ament_cmake REQUIRED)

# Questo comando copia la cartella 'launch' in install/share/tracking_launch/
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
