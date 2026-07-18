cmake_minimum_required(VERSION 3.8)
project(waypoint_mission)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wconversion -Wshadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(position_reporter_node src/node/position_reporter_node.cpp)
target_include_directories(position_reporter_node PRIVATE include)
target_include_directories(position_reporter_node PRIVATE Utils)
ament_target_dependencies(position_reporter_node rclcpp px4_msgs)

add_executable(offboard_supervisor_node src/core/offboard_supervisor.cpp src/node/offboard_supervisor_node.cpp)
target_include_directories(offboard_supervisor_node PRIVATE include)
target_include_directories(offboard_supervisor_node PRIVATE Utils)
ament_target_dependencies(offboard_supervisor_node rclcpp px4_msgs geometry_msgs std_msgs)

add_executable(waypoint_mission_node src/core/waypoint_mission.cpp src/node/waypoint_mission_node.cpp)
target_include_directories(waypoint_mission_node PRIVATE include)
target_include_directories(waypoint_mission_node PRIVATE Utils)
ament_target_dependencies(waypoint_mission_node rclcpp px4_msgs geometry_msgs std_msgs sensor_msgs)

add_executable(audit_topics tools/audit_topics.cpp)
ament_target_dependencies(audit_topics rclcpp)

install(TARGETS position_reporter_node offboard_supervisor_node waypoint_mission_node audit_topics
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

ament_package()
