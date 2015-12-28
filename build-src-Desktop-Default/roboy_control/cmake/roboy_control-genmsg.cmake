# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "roboy_control: 5 messages, 0 services")

set(MSG_I_FLAGS "-Iroboy_control:/home/matthias/roboycontrol/src/roboy_control/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(roboy_control_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg" NAME_WE)
add_custom_target(_roboy_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboy_control" "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg" NAME_WE)
add_custom_target(_roboy_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboy_control" "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg" NAME_WE)
add_custom_target(_roboy_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboy_control" "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg" NAME_WE)
add_custom_target(_roboy_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboy_control" "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg" NAME_WE)
add_custom_target(_roboy_control_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboy_control" "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
)
_generate_msg_cpp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
)
_generate_msg_cpp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
)
_generate_msg_cpp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
)
_generate_msg_cpp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
)

### Generating Services

### Generating Module File
_generate_module_cpp(roboy_control
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(roboy_control_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(roboy_control_generate_messages roboy_control_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_cpp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_cpp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_cpp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_cpp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_cpp _roboy_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboy_control_gencpp)
add_dependencies(roboy_control_gencpp roboy_control_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboy_control_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
)
_generate_msg_lisp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
)
_generate_msg_lisp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
)
_generate_msg_lisp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
)
_generate_msg_lisp(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
)

### Generating Services

### Generating Module File
_generate_module_lisp(roboy_control
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(roboy_control_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(roboy_control_generate_messages roboy_control_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_lisp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_lisp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_lisp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_lisp _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_lisp _roboy_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboy_control_genlisp)
add_dependencies(roboy_control_genlisp roboy_control_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboy_control_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
)
_generate_msg_py(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
)
_generate_msg_py(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
)
_generate_msg_py(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
)
_generate_msg_py(roboy_control
  "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
)

### Generating Services

### Generating Module File
_generate_module_py(roboy_control
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(roboy_control_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(roboy_control_generate_messages roboy_control_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeResponse.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_py _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/InitializeRequest.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_py _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Status.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_py _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Steer.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_py _roboy_control_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/matthias/roboycontrol/src/roboy_control/msg/Trajectory.msg" NAME_WE)
add_dependencies(roboy_control_generate_messages_py _roboy_control_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboy_control_genpy)
add_dependencies(roboy_control_genpy roboy_control_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboy_control_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboy_control
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(roboy_control_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboy_control
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(roboy_control_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboy_control
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(roboy_control_generate_messages_py std_msgs_generate_messages_py)
