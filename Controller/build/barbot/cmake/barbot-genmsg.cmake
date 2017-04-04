# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "barbot: 3 messages, 0 services")

set(MSG_I_FLAGS "-Ibarbot:/home/tommy/Workspace/BarBot/Controller/src/barbot/msg;-Igeometry_msgs:/opt/ros/kinetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(barbot_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" NAME_WE)
add_custom_target(_barbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barbot" "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" "geometry_msgs/Pose2D:std_msgs/Header"
)

get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" NAME_WE)
add_custom_target(_barbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barbot" "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" NAME_WE)
add_custom_target(_barbot_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "barbot" "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" "geometry_msgs/Pose2D:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barbot
)
_generate_msg_cpp(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barbot
)
_generate_msg_cpp(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barbot
)

### Generating Services

### Generating Module File
_generate_module_cpp(barbot
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barbot
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(barbot_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(barbot_generate_messages barbot_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" NAME_WE)
add_dependencies(barbot_generate_messages_cpp _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" NAME_WE)
add_dependencies(barbot_generate_messages_cpp _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" NAME_WE)
add_dependencies(barbot_generate_messages_cpp _barbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barbot_gencpp)
add_dependencies(barbot_gencpp barbot_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barbot_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barbot
)
_generate_msg_eus(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barbot
)
_generate_msg_eus(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barbot
)

### Generating Services

### Generating Module File
_generate_module_eus(barbot
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barbot
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(barbot_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(barbot_generate_messages barbot_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" NAME_WE)
add_dependencies(barbot_generate_messages_eus _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" NAME_WE)
add_dependencies(barbot_generate_messages_eus _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" NAME_WE)
add_dependencies(barbot_generate_messages_eus _barbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barbot_geneus)
add_dependencies(barbot_geneus barbot_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barbot_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barbot
)
_generate_msg_lisp(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barbot
)
_generate_msg_lisp(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barbot
)

### Generating Services

### Generating Module File
_generate_module_lisp(barbot
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barbot
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(barbot_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(barbot_generate_messages barbot_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" NAME_WE)
add_dependencies(barbot_generate_messages_lisp _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" NAME_WE)
add_dependencies(barbot_generate_messages_lisp _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" NAME_WE)
add_dependencies(barbot_generate_messages_lisp _barbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barbot_genlisp)
add_dependencies(barbot_genlisp barbot_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barbot_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barbot
)
_generate_msg_nodejs(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barbot
)
_generate_msg_nodejs(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barbot
)

### Generating Services

### Generating Module File
_generate_module_nodejs(barbot
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barbot
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(barbot_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(barbot_generate_messages barbot_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" NAME_WE)
add_dependencies(barbot_generate_messages_nodejs _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" NAME_WE)
add_dependencies(barbot_generate_messages_nodejs _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" NAME_WE)
add_dependencies(barbot_generate_messages_nodejs _barbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barbot_gennodejs)
add_dependencies(barbot_gennodejs barbot_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barbot_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot
)
_generate_msg_py(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot
)
_generate_msg_py(barbot
  "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/kinetic/share/geometry_msgs/cmake/../msg/Pose2D.msg;/opt/ros/kinetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot
)

### Generating Services

### Generating Module File
_generate_module_py(barbot
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(barbot_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(barbot_generate_messages barbot_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Waypoint.msg" NAME_WE)
add_dependencies(barbot_generate_messages_py _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/Thruster.msg" NAME_WE)
add_dependencies(barbot_generate_messages_py _barbot_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/tommy/Workspace/BarBot/Controller/src/barbot/msg/State.msg" NAME_WE)
add_dependencies(barbot_generate_messages_py _barbot_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(barbot_genpy)
add_dependencies(barbot_genpy barbot_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS barbot_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/barbot
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(barbot_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(barbot_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/barbot
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(barbot_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(barbot_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/barbot
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(barbot_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(barbot_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barbot)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/barbot
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(barbot_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(barbot_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/barbot
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(barbot_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(barbot_generate_messages_py std_msgs_generate_messages_py)
endif()
