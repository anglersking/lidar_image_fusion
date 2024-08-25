cmake_minimum_required(VERSION 2.8.3)
project(lidar_camera)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_BUILD_TYPE Release)

set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "-fPIC")
# work around for proj4
set({PROJ4_LIBRARIES proj)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
  rosbag
  roslib
  image_transport
  cv_bridge
  )

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(yaml-cpp REQUIRED)

#OpenCV
find_package( OpenCV 3.0 REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS})
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")



if(OPENMP_FOUND)
  set(OpenMP_LIBS gomp)
  set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(OpenCV REQUIRED)


include_directories(
  include
	${catkin_INCLUDE_DIRS} 
	#${EIGEN3_INCLUDE_DIR}
	${PCL_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    src
)

add_message_files(
  FILES
  calib_envluate.msg
)

generate_messages(
  DEPENDENCIES 
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL OpenCV
)





# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
             NAMES YAML_CPP
             PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})

if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")





add_definitions( -march=native )

LINK_DIRECTORIES(${LSTAR_DIR}/lib)


LINK_DIRECTORIES(${LSTAR_DIR}/lib)
add_library(io STATIC src/io.cpp src/evaluation.cpp src/plot.cc)
target_link_libraries(io ${catkin_LIBRARIES} ${YAML_CPP_LIBRARIES} )

add_executable(view_calib src/view_calib.cpp )
target_link_libraries(view_calib ${OpenCV_LIBS})
target_link_libraries(view_calib  io ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${YAML_CPP_LIBRARIES}  jsoncpp)

add_executable(fisheye2lidar src/fisheye2lidar.cpp src/io.cpp )
target_link_libraries(fisheye2lidar ${catkin_LIBRARIES} ${PCL_LIBRARIES}   ${YAML_CPP_LIBRARIES} jsoncpp)


