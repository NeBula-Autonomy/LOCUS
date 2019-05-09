cmake_minimum_required(VERSION 2.8.3)
project(blam_slam)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

FIND_PACKAGE(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  measurement_synchronizer
  point_cloud_filter
  point_cloud_odometry
  laser_loop_closure
  point_cloud_localization
  point_cloud_mapper
  geometry_utils
  parameter_utils
  pcl_ros
  pcl_conversions
  core_msgs
  geometry_msgs
  message_generation
)

add_service_files(FILES AddFactor.srv RemoveFactor.srv SaveGraph.srv)
generate_messages()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    message_runtime
    rosbag
    measurement_synchronizer
    point_cloud_filter
    point_cloud_odometry
    laser_loop_closure
    point_cloud_localization
    point_cloud_mapper
    geometry_utils
    parameter_utils
    pcl_ros
    pcl_conversions
    core_msgs
    geometry_msgs
  DEPENDS
    Boost
)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++11 support.")
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${GTSAM_INCLUDE_DIR} ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS} ${GTSAM_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/BlamSlam.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS})

add_executable(${PROJECT_NAME}_offline src/${PROJECT_NAME}_offline.cc)
target_link_libraries(${PROJECT_NAME}_offline
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)
