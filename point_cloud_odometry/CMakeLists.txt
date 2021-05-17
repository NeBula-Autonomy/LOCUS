cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_odometry)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

#SET(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.7 REQUIRED COMPONENTS common filters features registration)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif()

find_package(OpenMP)
# if (OpenMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
# endif()


find_package(catkin REQUIRED COMPONENTS
    diagnostic_msgs
    eigen_conversions
    geometry_msgs
    geometry_utils
    multithreaded_gicp
    parameter_utils
    pcl_ros
    roscpp
    sensor_msgs
    tf
    tf2_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    diagnostic_msgs
    eigen_conversions
    geometry_msgs
    geometry_utils
    multithreaded_gicp
    parameter_utils
    pcl_ros
    roscpp
    sensor_msgs
    tf
    tf2_ros  
)

# find_package(PCL 1.7 REQUIRED)
# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

# andrzej: this is oudated style i think
#link_directories(
#  ${catkin_LIBRARY_DIRS}
#  ${PCL_LIBRARY_DIRS}
#)

add_library(${PROJECT_NAME} src/PointCloudOdometry.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

#installation, #todo: verify whether we want to install it
#install(TARGETS ${PROJECT_NAME}
#  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
#)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  add_rostest_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})
endif()
