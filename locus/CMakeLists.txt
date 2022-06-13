cmake_minimum_required(VERSION 2.8.3)
project(locus)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++14)



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
  point_cloud_filter
  point_cloud_odometry
  point_cloud_localization
  point_cloud_mapper
  geometry_utils
  parameter_utils
  pcl_ros
  pcl_conversions
  core_msgs
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  visualization_msgs
  message_generation
  frontend_utils
  tf
  tf2_ros
  tf2_geometry_msgs
  tf2_sensor_msgs
  dynamic_reconfigure
  sensor_description
)

find_package(OpenMP)
# if (OpenMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
# endif()


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    message_runtime
    point_cloud_filter
    point_cloud_odometry
    point_cloud_localization
    point_cloud_mapper
    geometry_utils
    parameter_utils
    pcl_ros
    pcl_conversions
    core_msgs
    diagnostic_msgs
    geometry_msgs
    nav_msgs
    sensor_msgs
    std_msgs
    visualization_msgs
    frontend_utils
    tf
    tf2
    tf2_ros
    tf2_geometry_msgs
    tf2_sensor_msgs
    dynamic_reconfigure
    sensor_description
    message_filters
  DEPENDS
    Boost
)

#include(CheckCXXCompilerFlag)
#check_cxx_compiler_flag("-std=c++11" COMPILER_SUPPORTS_CXX11)
#check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
#if (COMPILER_SUPPORTS_CXX11)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#elseif(COMPILER_SUPPORTS_CXX0X)
#  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
#else()
#  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++11 support.")
#endif()

include_directories(include ${catkin_INCLUDE_DIRS}  ${Boost_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS}  ${Boost_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/Locus.cc)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}

)
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}

)

#############
## Testing ##
#############
if (CATKIN_ENABLE_TESTING)
find_package(rostest REQUIRED)
  # add_subdirectory(test)
  ## Add gtest based cpp test target and link libraries
  add_rostest_gtest(test_locus test/test_locus.test test/test_locus.cc)
  target_link_libraries(test_locus ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
