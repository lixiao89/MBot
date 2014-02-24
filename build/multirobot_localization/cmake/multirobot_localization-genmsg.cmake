# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "multirobot_localization: 1 messages, 1 services")

set(MSG_I_FLAGS "-Imultirobot_localization:/home/lixiao/Desktop/MBot/src/multirobot_localization/msg;-Igeometry_msgs:/home/lixiao/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg;-Istd_msgs:/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(multirobot_localization_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(multirobot_localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/msg/motor_ctr.msg"
  "${MSG_I_FLAGS}"
  "/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_localization
)

### Generating Services
_generate_srv_cpp(multirobot_localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/srv/Occupancy2D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_localization
)

### Generating Module File
_generate_module_cpp(multirobot_localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(multirobot_localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(multirobot_localization_generate_messages multirobot_localization_generate_messages_cpp)

# target for backward compatibility
add_custom_target(multirobot_localization_gencpp)
add_dependencies(multirobot_localization_gencpp multirobot_localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(multirobot_localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/msg/motor_ctr.msg"
  "${MSG_I_FLAGS}"
  "/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_localization
)

### Generating Services
_generate_srv_lisp(multirobot_localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/srv/Occupancy2D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_localization
)

### Generating Module File
_generate_module_lisp(multirobot_localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(multirobot_localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(multirobot_localization_generate_messages multirobot_localization_generate_messages_lisp)

# target for backward compatibility
add_custom_target(multirobot_localization_genlisp)
add_dependencies(multirobot_localization_genlisp multirobot_localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(multirobot_localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/msg/motor_ctr.msg"
  "${MSG_I_FLAGS}"
  "/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_localization
)

### Generating Services
_generate_srv_py(multirobot_localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/srv/Occupancy2D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_localization
)

### Generating Module File
_generate_module_py(multirobot_localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(multirobot_localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(multirobot_localization_generate_messages multirobot_localization_generate_messages_py)

# target for backward compatibility
add_custom_target(multirobot_localization_genpy)
add_dependencies(multirobot_localization_genpy multirobot_localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multirobot_localization_generate_messages_py)


debug_message(2 "multirobot_localization: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multirobot_localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(multirobot_localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multirobot_localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(multirobot_localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multirobot_localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(multirobot_localization_generate_messages_py geometry_msgs_generate_messages_py)
