cmake_minimum_required(VERSION 3.8)
project(swarm_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

# Make Python scripts executable
install(
  PROGRAMS scripts/swarm_monitor.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
