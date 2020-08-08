cmake_minimum_required(VERSION 2.8.3)
project(goals_sequence_path_planner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

## Generate messages in the 'msg' folder
add_message_files(DIRECTORY msg
  FILES
  PathArray.msg
  Node.msg
  NodeArray.msg
)

## Generate services in the 'srv' folder
add_service_files(DIRECTORY srv
  FILES
  SeqGoalsPathPlanner.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
)
