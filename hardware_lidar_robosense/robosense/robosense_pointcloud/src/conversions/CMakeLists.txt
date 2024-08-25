add_executable(robo_cloud_node cloud_node.cc convert.cc pointcloudXYZIR.cc)
add_dependencies(robo_cloud_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robo_cloud_node robosense_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS robo_cloud_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_library(robo_cloud_nodelet cloud_nodelet.cc convert.cc pointcloudXYZIR.cc)
add_dependencies(robo_cloud_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robo_cloud_nodelet robosense_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})
install(TARGETS robo_cloud_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})

add_executable(robo_bag_extractor bag_extractor.cc transform_extractor.cc)
add_dependencies(robo_bag_extractor ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robo_bag_extractor robosense_rawdata
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES})

add_executable(robo_transform_node transform_node.cc transform.cc pointcloudXYZIR.cc)
add_dependencies(robo_transform_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robo_transform_node robosense_rawdata mlog
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${GLOG_LIBRARIES})
                      
install(TARGETS robo_transform_node
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


add_library(robo_transform_nodelet transform_nodelet.cc transform.cc pointcloudXYZIR.cc)
add_dependencies(robo_transform_nodelet ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robo_transform_nodelet robosense_rawdata mlog
                      ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} ${GLOG_LIBRARIES})
install(TARGETS robo_transform_nodelet
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})


add_executable(robo_static_tf_node static_tf_node.cc)
add_dependencies(robo_static_tf_node ${${PROJECT_NAME}_EXPORTED_TARGETS})
target_link_libraries(robo_static_tf_node ${catkin_LIBRARIES} yaml-cpp)
