cmake_minimum_required(VERSION 2.8.3)
project(agent)

find_package(catkin REQUIRED COMPONENTS
  genmsg
  geometry_msgs
  message_generation
  roscpp
  rospy
  std_msgs
  actionlib
  actionlib_msgs
)

add_message_files(
  FILES
)

add_service_files(
FILES 
OpenGripperSrv.srv 
CloseGripperSrv.srv 
MoveToObject.srv 
MoveToButton.srv 
PressButtonSrv.srv 
ObtainObjectSrv.srv 
test.srv
)

add_action_files(DIRECTORY action FILES PressButtonAction.action)

generate_messages(
DEPENDENCIES 
std_msgs 
geometry_msgs
actionlib_msgs
)


catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

