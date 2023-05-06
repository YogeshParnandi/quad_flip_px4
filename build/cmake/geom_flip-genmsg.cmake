# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "geom_flip: 1 messages, 0 services")

set(MSG_I_FLAGS "-Igeom_flip:/home/yogesh/flip_ws/src/geom_flip/msg;-Igeometry_msgs:/opt/ros/noetic/share/geometry_msgs/cmake/../msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg;-Isensor_msgs:/opt/ros/noetic/share/sensor_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(geom_flip_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" NAME_WE)
add_custom_target(_geom_flip_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "geom_flip" "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" "geometry_msgs/Vector3:std_msgs/Header"
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(geom_flip
  "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geom_flip
)

### Generating Services

### Generating Module File
_generate_module_cpp(geom_flip
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geom_flip
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(geom_flip_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(geom_flip_generate_messages geom_flip_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" NAME_WE)
add_dependencies(geom_flip_generate_messages_cpp _geom_flip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geom_flip_gencpp)
add_dependencies(geom_flip_gencpp geom_flip_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geom_flip_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(geom_flip
  "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geom_flip
)

### Generating Services

### Generating Module File
_generate_module_eus(geom_flip
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geom_flip
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(geom_flip_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(geom_flip_generate_messages geom_flip_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" NAME_WE)
add_dependencies(geom_flip_generate_messages_eus _geom_flip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geom_flip_geneus)
add_dependencies(geom_flip_geneus geom_flip_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geom_flip_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(geom_flip
  "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geom_flip
)

### Generating Services

### Generating Module File
_generate_module_lisp(geom_flip
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geom_flip
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(geom_flip_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(geom_flip_generate_messages geom_flip_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" NAME_WE)
add_dependencies(geom_flip_generate_messages_lisp _geom_flip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geom_flip_genlisp)
add_dependencies(geom_flip_genlisp geom_flip_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geom_flip_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(geom_flip
  "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geom_flip
)

### Generating Services

### Generating Module File
_generate_module_nodejs(geom_flip
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geom_flip
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(geom_flip_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(geom_flip_generate_messages geom_flip_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" NAME_WE)
add_dependencies(geom_flip_generate_messages_nodejs _geom_flip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geom_flip_gennodejs)
add_dependencies(geom_flip_gennodejs geom_flip_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geom_flip_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(geom_flip
  "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/geometry_msgs/cmake/../msg/Vector3.msg;/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geom_flip
)

### Generating Services

### Generating Module File
_generate_module_py(geom_flip
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geom_flip
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(geom_flip_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(geom_flip_generate_messages geom_flip_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yogesh/flip_ws/src/geom_flip/msg/flatTargetmsg.msg" NAME_WE)
add_dependencies(geom_flip_generate_messages_py _geom_flip_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(geom_flip_genpy)
add_dependencies(geom_flip_genpy geom_flip_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS geom_flip_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geom_flip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/geom_flip
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(geom_flip_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(geom_flip_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(geom_flip_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geom_flip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/geom_flip
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_eus)
  add_dependencies(geom_flip_generate_messages_eus geometry_msgs_generate_messages_eus)
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(geom_flip_generate_messages_eus std_msgs_generate_messages_eus)
endif()
if(TARGET sensor_msgs_generate_messages_eus)
  add_dependencies(geom_flip_generate_messages_eus sensor_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geom_flip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/geom_flip
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(geom_flip_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(geom_flip_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(geom_flip_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geom_flip)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/geom_flip
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_nodejs)
  add_dependencies(geom_flip_generate_messages_nodejs geometry_msgs_generate_messages_nodejs)
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(geom_flip_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()
if(TARGET sensor_msgs_generate_messages_nodejs)
  add_dependencies(geom_flip_generate_messages_nodejs sensor_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geom_flip)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geom_flip\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/geom_flip
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(geom_flip_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(geom_flip_generate_messages_py std_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(geom_flip_generate_messages_py sensor_msgs_generate_messages_py)
endif()
