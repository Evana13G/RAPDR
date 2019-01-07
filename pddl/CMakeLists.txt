cmake_minimum_required(VERSION 2.8.3)
project(pddl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  rospack
  message_generation
  genmsg
)

add_message_files(
FILES
)

add_service_files(
  FILES 
  PlanExecutorSrv.srv 
)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
