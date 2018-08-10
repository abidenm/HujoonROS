# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "SkillMate: 2 messages, 0 services")

set(MSG_I_FLAGS "-ISkillMate:/home/ckim/HujoonROS/src/SkillMate/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(SkillMate_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg" NAME_WE)
add_custom_target(_SkillMate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "SkillMate" "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg" ""
)

get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg" NAME_WE)
add_custom_target(_SkillMate_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "SkillMate" "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(SkillMate
  "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/SkillMate
)
_generate_msg_cpp(SkillMate
  "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/SkillMate
)

### Generating Services

### Generating Module File
_generate_module_cpp(SkillMate
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/SkillMate
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(SkillMate_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(SkillMate_generate_messages SkillMate_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg" NAME_WE)
add_dependencies(SkillMate_generate_messages_cpp _SkillMate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg" NAME_WE)
add_dependencies(SkillMate_generate_messages_cpp _SkillMate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(SkillMate_gencpp)
add_dependencies(SkillMate_gencpp SkillMate_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS SkillMate_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(SkillMate
  "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/SkillMate
)
_generate_msg_lisp(SkillMate
  "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/SkillMate
)

### Generating Services

### Generating Module File
_generate_module_lisp(SkillMate
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/SkillMate
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(SkillMate_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(SkillMate_generate_messages SkillMate_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg" NAME_WE)
add_dependencies(SkillMate_generate_messages_lisp _SkillMate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg" NAME_WE)
add_dependencies(SkillMate_generate_messages_lisp _SkillMate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(SkillMate_genlisp)
add_dependencies(SkillMate_genlisp SkillMate_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS SkillMate_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(SkillMate
  "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/SkillMate
)
_generate_msg_py(SkillMate
  "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/SkillMate
)

### Generating Services

### Generating Module File
_generate_module_py(SkillMate
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/SkillMate
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(SkillMate_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(SkillMate_generate_messages SkillMate_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/Mouse3dCommand.msg" NAME_WE)
add_dependencies(SkillMate_generate_messages_py _SkillMate_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/ckim/HujoonROS/src/SkillMate/msg/HapticCommand.msg" NAME_WE)
add_dependencies(SkillMate_generate_messages_py _SkillMate_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(SkillMate_genpy)
add_dependencies(SkillMate_genpy SkillMate_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS SkillMate_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/SkillMate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/SkillMate
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(SkillMate_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/SkillMate)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/SkillMate
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(SkillMate_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/SkillMate)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/SkillMate\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/SkillMate
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(SkillMate_generate_messages_py std_msgs_generate_messages_py)
endif()
