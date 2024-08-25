# build the driver node
add_executable(robosense_node robosense_node.cc driver.cc)
add_dependencies(robosense_node robosense_driver_gencfg)
target_link_libraries(robosense_node
  robosense_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# build the nodelet version
add_library(robo_driver_nodelet nodelet.cc driver.cc)
add_dependencies(robo_driver_nodelet robosense_driver_gencfg)
target_link_libraries(robo_driver_nodelet
  robosense_input
  ${catkin_LIBRARIES}
  ${libpcap_LIBRARIES}
)

# install runtime files
install(TARGETS robosense_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        COMPONENT main
)
install(TARGETS robo_driver_nodelet
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
