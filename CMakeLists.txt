cmake_minimum_required(VERSION 2.8.3)
project(laneitri)

## Compile as C++11, supported in ROS Kinetic and newer
 add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  nav_msgs
  nmea_msgs
  cv_bridge
  image_transport
  pcl_conversions
  #pcl_common
  pcl_ros 
  #tf2_geometry_msgs
  geometry_msgs
  tf
  eigen_conversions
  tf2
  tf2_ros
  tf_conversions
  message_generation
  visualization_msgs
  pinhole_projection
  #Iterative-Closest-Point
)
#find_package(catkin REQUIRED COMPONENTS )
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
 add_message_files(
   FILES
   VelodyneTelemetry.msg
)
generate_messages(
  DEPENDENCIES
  geometry_msgs
  nav_msgs
  nmea_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
)
#cmake -DCMAKE_BUILD_TYPE=Release ..
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wno-unused-private-field")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb ")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")



catkin_package(
  INCLUDE_DIRS include
  LIBRARIES lanetracking
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime
  #DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


set(YOUR_LIB_SOURCES
        ${CMAKE_CURRENT_SOURCE_DIR}/src/parabolicMask.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/LaneModels.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/LaneDetection.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Tracking.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Test.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Processing.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Local.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/Hungarian.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/svd.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/KdTree.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/src/ICP.cpp
        ${CMAKE_CURRENT_SOURCE_DIR}/include/Hungarian.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/svd.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/KdTree.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/ICP.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/defs_and_types.h
        ${CMAKE_CURRENT_SOURCE_DIR}/include/LaneDetection.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/LaneModels.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/parabolicMask.h
	${CMAKE_CURRENT_SOURCE_DIR}/include/random_unique.hpp
	${CMAKE_CURRENT_SOURCE_DIR}/include/RANSAC.hpp
)
add_executable(laneitri_node src/main.cpp)

add_library(laneitri ${YOUR_LIB_SOURCES}  )





 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )



target_link_libraries(laneitri_node
   laneitri
   ${catkin_LIBRARIES}
   ${OpenCV_LIBS}
   ${CMAKE_THREAD_LIBS_INIT}
   
   ${INCLUDE_DIR}
 )



 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )

