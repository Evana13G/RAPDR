# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "action_primitive_variation: 1 messages, 2 services")

set(MSG_I_FLAGS "-Iaction_primitive_variation:/home/Mateo/ros_ws/src/action_primitive_variation/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(action_primitive_variation_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" NAME_WE)
add_custom_target(_action_primitive_variation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_primitive_variation" "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" ""
)

get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" NAME_WE)
add_custom_target(_action_primitive_variation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_primitive_variation" "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" ""
)

get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" NAME_WE)
add_custom_target(_action_primitive_variation_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "action_primitive_variation" "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_primitive_variation
)

### Generating Services
_generate_srv_cpp(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_primitive_variation
)
_generate_srv_cpp(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_primitive_variation
)

### Generating Module File
_generate_module_cpp(action_primitive_variation
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_primitive_variation
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(action_primitive_variation_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(action_primitive_variation_generate_messages action_primitive_variation_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_cpp _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_cpp _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_cpp _action_primitive_variation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_primitive_variation_gencpp)
add_dependencies(action_primitive_variation_gencpp action_primitive_variation_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_primitive_variation_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_primitive_variation
)

### Generating Services
_generate_srv_eus(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_primitive_variation
)
_generate_srv_eus(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_primitive_variation
)

### Generating Module File
_generate_module_eus(action_primitive_variation
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_primitive_variation
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(action_primitive_variation_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(action_primitive_variation_generate_messages action_primitive_variation_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_eus _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_eus _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_eus _action_primitive_variation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_primitive_variation_geneus)
add_dependencies(action_primitive_variation_geneus action_primitive_variation_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_primitive_variation_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_primitive_variation
)

### Generating Services
_generate_srv_lisp(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_primitive_variation
)
_generate_srv_lisp(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_primitive_variation
)

### Generating Module File
_generate_module_lisp(action_primitive_variation
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_primitive_variation
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(action_primitive_variation_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(action_primitive_variation_generate_messages action_primitive_variation_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_lisp _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_lisp _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_lisp _action_primitive_variation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_primitive_variation_genlisp)
add_dependencies(action_primitive_variation_genlisp action_primitive_variation_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_primitive_variation_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_primitive_variation
)

### Generating Services
_generate_srv_nodejs(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_primitive_variation
)
_generate_srv_nodejs(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_primitive_variation
)

### Generating Module File
_generate_module_nodejs(action_primitive_variation
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_primitive_variation
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(action_primitive_variation_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(action_primitive_variation_generate_messages action_primitive_variation_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_nodejs _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_nodejs _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_nodejs _action_primitive_variation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_primitive_variation_gennodejs)
add_dependencies(action_primitive_variation_gennodejs action_primitive_variation_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_primitive_variation_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation
)

### Generating Services
_generate_srv_py(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation
)
_generate_srv_py(action_primitive_variation
  "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation
)

### Generating Module File
_generate_module_py(action_primitive_variation
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(action_primitive_variation_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(action_primitive_variation_generate_messages action_primitive_variation_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/msg/Num.msg" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_py _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/PushButton.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_py _action_primitive_variation_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/Mateo/ros_ws/src/action_primitive_variation/srv/MoveArm.srv" NAME_WE)
add_dependencies(action_primitive_variation_generate_messages_py _action_primitive_variation_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(action_primitive_variation_genpy)
add_dependencies(action_primitive_variation_genpy action_primitive_variation_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS action_primitive_variation_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_primitive_variation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/action_primitive_variation
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(action_primitive_variation_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_primitive_variation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/action_primitive_variation
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(action_primitive_variation_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_primitive_variation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/action_primitive_variation
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(action_primitive_variation_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_primitive_variation)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/action_primitive_variation
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(action_primitive_variation_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/action_primitive_variation
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(action_primitive_variation_generate_messages_py std_msgs_generate_messages_py)
endif()
