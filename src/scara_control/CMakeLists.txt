cmake_minimum_required(VERSION 3.8)
project(scara_control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(controller_manager REQUIRED)
find_package(control_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(ros2_control REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(xacro REQUIRED)
find_package(robot_state_publisher REQUIRED)
find_package(gazebo_ros REQUIRED)

# Build C++ node
add_executable(coordination_node src/coordination_node.cpp)
ament_target_dependencies(coordination_node rclcpp std_msgs geometry_msgs sensor_msgs)

install(TARGETS coordination_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  src/target_generator.py
  src/spawn_objects.py
  src/gripper_control.py
  src/conveyor_control.py
  src/ik_solver.py
  src/joint_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

