cmake_minimum_required(VERSION 2.8)

if (DEFINED MLOG_NAMESPACE)
   set(MLOG_PROJECT_NAME "${MLOG_NAMESPACE}_mlog")
else()
   set(MLOG_PROJECT_NAME mlog)
endif()

set(MLOG_CURRENT_VERSION "1.2")

if (NOT TARGET ${MLOG_PROJECT_NAME})
  set(CMAKE_CXX_STANDARD 11)
  project (${MLOG_PROJECT_NAME})

  option(MLOG_ENABLE_ROS "Enable ROS functions" OFF)
  message(STATUS "MLOG_ENABLE_ROS=" ${MLOG_ENABLE_ROS})

  option(MLOG_ENABLE_MDC_ROS "Enable ROS functions on Huawei MDC" OFF)
  message(STATUS "MLOG_ENABLE_MDC_ROS=" ${MLOG_ENABLE_MDC_ROS})

  option(MLOG_ENABLE_MFR "Enable MFR functions" OFF)
  message(STATUS "MLOG_ENABLE_MFR=" ${MLOG_ENABLE_MFR})

  option(MLOG_ENABLE_PROFILING "Enable time profiling functions" OFF)
  message(STATUS "MLOG_ENABLE_PROFILING=" ${MLOG_ENABLE_PROFILING})

  option(MACHINE_ALREADY_EXIST "MACHINE_ALREADY_EXIST" OFF)
  message(STATUS "MACHINE_ALREADY_EXIST=" ${MACHINE_ALREADY_EXIST})

  set(MLOG_MSG_ID_LIB ${PROJECT_NAME}_msg_id)
  add_library(${MLOG_MSG_ID_LIB} INTERFACE)
  target_include_directories(${MLOG_MSG_ID_LIB} INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/msg_id/include/>
    $<INSTALL_INTERFACE:include>
  )

  file(GLOB_RECURSE SRC_FILES ${CMAKE_CURRENT_SOURCE_DIR}/core/src/**)
  add_library(${PROJECT_NAME} STATIC ${SRC_FILES})
  set_target_properties(${PROJECT_NAME} PROPERTIES POSITION_INDEPENDENT_CODE ON)
  set_target_properties(${PROJECT_NAME} PROPERTIES VERSION ${MLOG_CURRENT_VERSION})
  target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/core/include/>
    $<INSTALL_INTERFACE:include>
  )
  target_link_libraries(${PROJECT_NAME} PUBLIC ${MLOG_MSG_ID_LIB})

  if (MLOG_ENABLE_ROS)
    target_compile_definitions(${MLOG_MSG_ID_LIB} INTERFACE -DMLOG_ENABLE_ROS)
    find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)  # Only those necessary for mlog_msg_id lib
    target_include_directories(${MLOG_MSG_ID_LIB} INTERFACE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${MLOG_MSG_ID_LIB} INTERFACE ${catkin_LIBRARIES})

    add_subdirectory(ros/mlog_msgs)
    # Find again with full ROS requirements
    find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs message_generation mlog_msgs)
    target_include_directories(${PROJECT_NAME} PRIVATE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${catkin_LIBRARIES})
    add_dependencies(${PROJECT_NAME} mlog_msgs_generate_messages_cpp)
  endif()

  if (MLOG_ENABLE_MDC_ROS)
    target_compile_definitions(${MLOG_MSG_ID_LIB} INTERFACE -DMLOG_ENABLE_ROS -DMLOG_ENABLE_MDC_ROS)
    list(APPEND HISI_ROS
            rt
            cpp_common
            rosconsole
            roscpp
            roscpp_serialization
            ascendcl
            rostime
            rostimer
            rostopic_tool
            topic_tools
            cm_shm
            param_shm
            camera_cmi
            camera_rpmsg
            xmlrpcpp
            log
            ros1cpp
            ros_type_introspection
            queue_schedule
            easy_comm
            hiperf
            iam
            runtime
            ros_fc)

    set(MDC_SDK "/usr/local/mdc_cross_compile_sdk")

    foreach (ROS_TARGET IN LISTS HISI_ROS)
        find_library(${ROS_TARGET}_LIB PATHS
                ${MDC_SDK}/sysroot/usr/lib64
                ${MDC_SDK}/sysroot/usr/local/Ascend/acllib/lib64
                ${MDC_SDK}/sysroot/usr/lib/mdc_ros/hiva/cm
                ${MDC_SDK}/sysroot/usr/lib/mdc_ros/media
                ${MDC_SDK}/sysroot/usr/lib
                ${MDC_SDK}/sysroot/usr/lib64
                ${MDC_SDK}/sysroot/usr/lib/mdc/base-plat
                NAMES ${ROS_TARGET})
        list(APPEND HISI_ROS_LIBS ${${ROS_TARGET}_LIB})
    endforeach ()

    target_include_directories(${PROJECT_NAME} PRIVATE ${MDC_SDK}/sysroot/usr/include/mdc_ros/hiva/cm
            ${MDC_SDK}/sysroot/usr/include/mdc_ros/driver
            ${MDC_SDK}/sysroot/usr/include/mdc_ros/driver/driver
            ${MDC_SDK}/sysroot/usr/include/mdc_ros/aos
            ${MDC_SDK}/sysroot/usr/include/mdc_ros
            ${CMAKE_CURRENT_LIST_DIR}/ros/include_mdc)
    target_link_libraries(${PROJECT_NAME} PRIVATE ${HISI_ROS_LIBS})
  endif ()

  if (MLOG_ENABLE_MFR)    
    target_compile_options(${PROJECT_NAME} PUBLIC -DMLOG_ENABLE_MFR=ON)
    if (MACHINE_ALREADY_EXIST)
      target_compile_options(${PROJECT_NAME} PUBLIC -DMACHINE_ALREADY_EXIST=ON)
    endif()
    if(NOT TARGET ${mlog_mfrmsgs_LIBRARIES})
      include(${MFRUNTIME_PATH}/.cmake/generate_mfrmessages.cmake)
    
      include(./std_mfrmsgs/mfrmsg.cmake)
      include(./mfr/mlog_mfrmsgs/mfrmsg.cmake)
      
      find_library(MFRUNTIME libmfruntime.so ${MFRUNTIME_BUILD_PATH}/)
      target_link_libraries(${PROJECT_NAME} ${MFRUNTIME})
          
      target_link_libraries(${PROJECT_NAME}
        ${generate_mfr_LIBRARIES}
      )
      target_include_directories(${PROJECT_NAME} PRIVATE ${generate_mfr_INCLUDE_DIRS})
        
      set(INCLUDE ${MFRUNTIME_PATH}/include)
      target_include_directories(${PROJECT_NAME} PRIVATE $<BUILD_INTERFACE:${INCLUDE}>)
    endif()
  endif()

  if (MLOG_ENABLE_PROFILING) 
    target_compile_options(${PROJECT_NAME} PUBLIC -DMLOG_ENABLE_PROFILING=ON)
  endif()

  ## install
  
  set(CMAKE_INSTALL_PREFIX ${CMAKE_CURRENT_BINARY_DIR}/install/)
  install(DIRECTORY
    ${PROJECT_SOURCE_DIR}/msg_id/include/
    ${PROJECT_SOURCE_DIR}/core/include/
    DESTINATION include
    FILES_MATCHING PATTERN "*"
  )
  install(TARGETS ${MLOG_MSG_ID_LIB} DESTINATION lib EXPORT ${MLOG_MSG_ID_LIB}_lib)
  install(EXPORT ${MLOG_MSG_ID_LIB}_lib
    FILE ${MLOG_MSG_ID_LIB}Config.cmake
    DESTINATION cmake)

  if (MLOG_ENABLE_MFR)
    install(TARGETS ${PROJECT_NAME} ${std_mfrmsgs_LIBRARIES} ${mlog_mfrmsgs_LIBRARIES} DESTINATION lib EXPORT ${MLOG_PROJECT_NAME}_lib)
  else()
    install(TARGETS ${PROJECT_NAME} DESTINATION lib EXPORT ${MLOG_PROJECT_NAME}_lib)
  endif()
  
  install(EXPORT ${MLOG_PROJECT_NAME}_lib
    FILE ${MLOG_PROJECT_NAME}Config.cmake
    DESTINATION cmake)

  if (DEFINED MLOG_NAMESPACE)
    add_library(${MLOG_NAMESPACE}::mlog ALIAS ${MLOG_PROJECT_NAME})
    add_library(${MLOG_NAMESPACE}::mlog_msg_id ALIAS ${MLOG_MSG_ID_LIB})
  endif()
else()
  get_target_property(FOUND_MLOG_VERSION ${MLOG_PROJECT_NAME} VERSION)
  if (NOT FOUND_MLOG_VERSION EQUAL "${MLOG_CURRENT_VERSION}")
     message(WARNING "Link to another existed '${MLOG_PROJECT_NAME}' target (version ${FOUND_MLOG_VERSION}), which might not be the version (${MLOG_CURRENT_VERSION}) you wished.")
  endif()
endif()

if (${CMAKE_CURRENT_SOURCE_DIR} STREQUAL ${CMAKE_SOURCE_DIR})
  add_subdirectory(sample/cpp)
endif()
