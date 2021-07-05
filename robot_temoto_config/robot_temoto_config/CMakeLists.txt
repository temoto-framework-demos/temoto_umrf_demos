cmake_minimum_required(VERSION 2.8.3)
project(robot_temoto_config)

# use c++11 standard
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  temoto_action_assistant
  message_generation
)

add_service_files(FILES
  SetChargeMode.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(battery_charge_node
  src/battery_charge_node.cpp
)

add_dependencies(battery_charge_node
  ${catkin_EXPORTED_TARGETS}
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
)

target_link_libraries(battery_charge_node
  ${catkin_LIBRARIES}
  ${libraries}
)