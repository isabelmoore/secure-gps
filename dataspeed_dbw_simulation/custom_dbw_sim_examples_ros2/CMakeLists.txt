cmake_minimum_required(VERSION 3.5)
project(custom_dbw_sim_examples)

find_package(ament_cmake_ros REQUIRED)

install(
  DIRECTORY
    launch
    rviz
    urdf
    worlds
    yaml
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/multi_car_cmd.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()