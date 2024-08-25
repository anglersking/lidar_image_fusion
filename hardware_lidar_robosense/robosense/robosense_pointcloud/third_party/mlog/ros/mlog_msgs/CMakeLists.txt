cmake_minimum_required(VERSION 2.8.3)
project(mlog_msgs)

set(MLOG_MSGS_CURRENT_VERSION "1.0")

if (NOT TARGET mlog_msgs_generate_messages)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
  )
  
  add_message_files(DIRECTORY msg/mlog_msg NOINSTALL)
  generate_messages(
    DEPENDENCIES
    std_msgs
  )

  catkin_package(
    CATKIN_DEPENDS message_runtime roscpp rospy std_msgs
  )

  include_directories(
    ${catkin_INCLUDE_DIRS}
  )
  install(DIRECTORY msg/mlog_msg/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/msg)

  set_target_properties(mlog_msgs_generate_messages PROPERTIES VERSION ${MLOG_MSGS_CURRENT_VERSION})
else()
  get_target_property(FOUND_MLOG_MSGS_VERSION mlog_msgs_generate_messages VERSION)
  if (NOT FOUND_MLOG_MSGS_VERSION EQUAL "${MLOG_MSGS_CURRENT_VERSION}")
     message(WARNING "Link to another existed mlog_msgs(version ${FOUND_MLOG_MSGS_VERSION}), which might not be the version (${MLOG_MSGS_CURRENT_VERSION}) you wished.")
  endif()
endif()
