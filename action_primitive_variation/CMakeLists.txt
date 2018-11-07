cmake_minimum_required(VERSION 2.8.3)
project(action_primitive_variation)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs genmsg message_generation)

## Declare ROS messages and services
add_message_files(FILES Num.msg)
add_service_files(FILES OpenGripper.srv CloseGripper.srv MoveToObject.srv MoveToButton.srv PressButtonSrv.srv GraspWithGripper.srv PushButton.srv test.srv)

## Generate added messages and services
generate_messages(DEPENDENCIES std_msgs geometry_msgs)

## Declare a catkin package
catkin_package()

## Build talker and listener
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(push_button_server src/push_button_server.cpp)
target_link_libraries(push_button_server ${catkin_LIBRARIES})
add_dependencies(push_button_server action_primitive_variation_gencpp)

add_executable(push_button_client src/push_button_client.cpp)
target_link_libraries(push_button_client ${catkin_LIBRARIES})
add_dependencies(push_button_client action_primitive_variation_gencpp)
