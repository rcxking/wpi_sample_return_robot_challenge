# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "fovis_ros: 1 messages, 0 services")

set(MSG_I_FLAGS "-Ifovis_ros:/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/fovis_ros/msg;-Istd_msgs:/opt/ros/hydro/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(fovis_ros_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(fovis_ros
  "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/fovis_ros/msg/FovisInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fovis_ros
)

### Generating Services

### Generating Module File
_generate_module_cpp(fovis_ros
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fovis_ros
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(fovis_ros_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(fovis_ros_generate_messages fovis_ros_generate_messages_cpp)

# target for backward compatibility
add_custom_target(fovis_ros_gencpp)
add_dependencies(fovis_ros_gencpp fovis_ros_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fovis_ros_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(fovis_ros
  "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/fovis_ros/msg/FovisInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fovis_ros
)

### Generating Services

### Generating Module File
_generate_module_lisp(fovis_ros
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fovis_ros
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(fovis_ros_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(fovis_ros_generate_messages fovis_ros_generate_messages_lisp)

# target for backward compatibility
add_custom_target(fovis_ros_genlisp)
add_dependencies(fovis_ros_genlisp fovis_ros_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fovis_ros_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(fovis_ros
  "/home/will/Code/wpi-sample-return-robot-challenge/rockie_code/src/fovis_ros/msg/FovisInfo.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/hydro/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fovis_ros
)

### Generating Services

### Generating Module File
_generate_module_py(fovis_ros
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fovis_ros
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(fovis_ros_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(fovis_ros_generate_messages fovis_ros_generate_messages_py)

# target for backward compatibility
add_custom_target(fovis_ros_genpy)
add_dependencies(fovis_ros_genpy fovis_ros_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS fovis_ros_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fovis_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/fovis_ros
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(fovis_ros_generate_messages_cpp std_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fovis_ros)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/fovis_ros
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(fovis_ros_generate_messages_lisp std_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fovis_ros)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fovis_ros\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/fovis_ros
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(fovis_ros_generate_messages_py std_msgs_generate_messages_py)
