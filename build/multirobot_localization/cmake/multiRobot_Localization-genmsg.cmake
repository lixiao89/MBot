# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "multiRobot_Localization: 1 messages, 1 services")

set(MSG_I_FLAGS "-ImultiRobot_Localization:/home/lixiao/Desktop/MBot/src/multirobot_localization/msg;-Igeometry_msgs:/home/lixiao/ros_catkin_ws/install_isolated/share/geometry_msgs/cmake/../msg;-Istd_msgs:/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(multiRobot_Localization_generate_messages ALL)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(multiRobot_Localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/msg/motor_ctr.msg"
  "${MSG_I_FLAGS}"
  "/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiRobot_Localization
)

### Generating Services
_generate_srv_cpp(multiRobot_Localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/srv/Occupancy2D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiRobot_Localization
)

### Generating Module File
_generate_module_cpp(multiRobot_Localization
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiRobot_Localization
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(multiRobot_Localization_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(multiRobot_Localization_generate_messages multiRobot_Localization_generate_messages_cpp)

# target for backward compatibility
add_custom_target(multiRobot_Localization_gencpp)
add_dependencies(multiRobot_Localization_gencpp multiRobot_Localization_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiRobot_Localization_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(multiRobot_Localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/msg/motor_ctr.msg"
  "${MSG_I_FLAGS}"
  "/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiRobot_Localization
)

### Generating Services
_generate_srv_lisp(multiRobot_Localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/srv/Occupancy2D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiRobot_Localization
)

### Generating Module File
_generate_module_lisp(multiRobot_Localization
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiRobot_Localization
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(multiRobot_Localization_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(multiRobot_Localization_generate_messages multiRobot_Localization_generate_messages_lisp)

# target for backward compatibility
add_custom_target(multiRobot_Localization_genlisp)
add_dependencies(multiRobot_Localization_genlisp multiRobot_Localization_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiRobot_Localization_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(multiRobot_Localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/msg/motor_ctr.msg"
  "${MSG_I_FLAGS}"
  "/home/lixiao/ros_catkin_ws/install_isolated/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiRobot_Localization
)

### Generating Services
_generate_srv_py(multiRobot_Localization
  "/home/lixiao/Desktop/MBot/src/multirobot_localization/srv/Occupancy2D.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiRobot_Localization
)

### Generating Module File
_generate_module_py(multiRobot_Localization
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiRobot_Localization
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(multiRobot_Localization_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(multiRobot_Localization_generate_messages multiRobot_Localization_generate_messages_py)

# target for backward compatibility
add_custom_target(multiRobot_Localization_genpy)
add_dependencies(multiRobot_Localization_genpy multiRobot_Localization_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS multiRobot_Localization_generate_messages_py)


debug_message(2 "multiRobot_Localization: Iflags=${MSG_I_FLAGS}")


if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiRobot_Localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/multiRobot_Localization
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
add_dependencies(multiRobot_Localization_generate_messages_cpp geometry_msgs_generate_messages_cpp)

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiRobot_Localization)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/multiRobot_Localization
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
add_dependencies(multiRobot_Localization_generate_messages_lisp geometry_msgs_generate_messages_lisp)

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiRobot_Localization)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiRobot_Localization\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/multiRobot_Localization
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
add_dependencies(multiRobot_Localization_generate_messages_py geometry_msgs_generate_messages_py)
