cmake_minimum_required(VERSION 2.8.3)
project(environment)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  rospack
  roscpp
  rospack
  rospy
  std_msgs
  message_generation
  geometry_msgs
  genmsg
)

add_message_files(
FILES
  Num.msg
  At.msg
  State.msg
  Predicate.msg
  PredicateList.msg
)

add_service_files(
  FILES 
  ScenarioDataSrv.srv 
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs gazebo_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

