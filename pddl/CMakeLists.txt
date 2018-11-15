cmake_minimum_required(VERSION 2.8.3)
project(pddl)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  ## rosplan_planning_system
  ## diagnostic_msgs
  ## rosplan_dispatch_msgs
  ## rosplan_knowledge_msgs
)

## Compile as C++11, supported in ROS Kinetic and newer
## add_compile_options(-std=c++11)


## Declare things to be passed to dependent projects
## catkin_package(
##   INCLUDE_DIRS include ${catkin_INCLUDE_DIRS}
##   LIBRARIES rosplan_action_interface
##   CATKIN_DEPENDS roscpp rospy std_msgs diagnostic_msgs rosplan_dispatch_msgs rosplan_knowledge_msgs
##   DEPENDS
## )

###########
## Build ##
###########

## include_directories(include)
## include_directories(${catkin_INCLUDE_DIRS}



## Declare action interface for press_button action interface
## add_executable(pressButtonInterface src/PressButtonClient.cpp ${rosplan_planning_system}/src/ActionInterface/RPActionInterface.cpp)
## add_dependencies(pressButtonInterface ${catkin_EXPORTED_TARGETS})
## target_link_libraries(pressButtonInterface ${catkin_LIBRARIES})

## Declare action interface for obtain_object action interface
## add_executable(obtainObjectInterface src/ObtainObjectClient.cpp ${rosplan_planning_system}/src/ActionInterface/RPActionInterface.cpp)
## add_dependencies(obtainObjectInterface ${catkin_EXPORTED_TARGETS})
## target_link_libraries(obtainObjectInterface ${catkin_LIBRARIES})