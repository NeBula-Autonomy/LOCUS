cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_filter)

find_package(PCL REQUIRED COMPONENTS common filters)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif(NOT PCL_FOUND)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  parameter_utils
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    parameter_utils
    pcl_ros
  DEPENDS
    pcl
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/PointCloudFilter.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
