add_library(robosense_input input.cc)
target_link_libraries(robosense_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(robosense_input ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS robosense_input
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
