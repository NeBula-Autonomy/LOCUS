cmake_minimum_required(VERSION 2.8.3)
project(laser_loop_closure)

find_package(PCL REQUIRED COMPONENTS common registration)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif(NOT PCL_FOUND)

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_utils
  parameter_utils
  point_cloud_filter
  pose_graph_msgs
  visualization_msgs
  std_msgs
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    geometry_utils
    parameter_utils
    point_cloud_filter
    pose_graph_msgs
    visualization_msgs
    std_msgs
    pcl_ros
  DEPENDS
    PCL
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

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/LaserLoopClosure.cc)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)
