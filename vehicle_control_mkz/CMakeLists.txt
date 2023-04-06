cmake_minimum_required(VERSION 3.5)
project(vehicle_control_mkz)

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

# DEPENDENCIES
find_package(ament_cmake REQUIRED)
find_package(dbw_gazebo_mkz REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(dbw_ford_msgs)

#MSG- Location of odom msg, #scripts- run odompubtest.py for odom generation 
install(DIRECTORY
  scripts
  msg
  vehicle_control_mkz
  launch
  DESTINATION share/${PROJECT_NAME}/
)
#for testing YAML
install(DIRECTORY
  config
  DESTINATION lib/${PROJECT_NAME})
  


# Install Python modules
ament_python_install_package(${PROJECT_NAME})
 
# Install Python executables
install(PROGRAMS
  scripts/odompubtest_mkz.py
  scripts/collect_waypoints.py
  scripts/lat_control.py
  scripts/long_control.py
  scripts/steering_methods.py
  scripts/ros_callback.py
  scripts/pid.py
  scripts/low_pass_filter.py
  DESTINATION lib/${PROJECT_NAME}
)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/A9.msg"
   DEPENDENCIES builtin_interfaces std_msgs
 )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
