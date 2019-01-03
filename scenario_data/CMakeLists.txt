cmake_minimum_required(VERSION 2.8.3)
project(scenario_data)

find_package(catkin REQUIRED COMPONENTS
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
)

add_service_files(
FILES 
ScenarioDataSrv.srv 
)

# catkin_python_setup()

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)