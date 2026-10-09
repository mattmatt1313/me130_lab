cmake_minimum_required(VERSION 3.16)
project(me130_wheel)

find_package(ament_cmake REQUIRED)

# The controller is Python so students can edit it without a C++ rebuild.
# Reachable as: ros2 run me130_wheel wheel_speed_pi.py
install(PROGRAMS
  scripts/wheel_speed_pi.py
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

ament_package()
