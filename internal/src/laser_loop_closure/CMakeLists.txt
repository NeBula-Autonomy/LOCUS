cmake_minimum_required(VERSION 3.5)
project(laser_loop_closure)

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX14)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX14)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++14 support.")
endif()

find_package(GTSAM REQUIRED)
if (NOT GTSAM_FOUND)
  message(FATAL_ERROR "This program requires the GTSAM library.")
endif(NOT GTSAM_FOUND)

find_package(PCL 1.7 REQUIRED)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif(NOT PCL_FOUND)

find_package(Boost 1.58 COMPONENTS filesystem program_options timer REQUIRED)

find_package(ZLIB REQUIRED)

find_package(SESync REQUIRED)

# Need to do this to prevent Eigen conflict with SESync
if (NOT TARGET rosComponents)
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_utils
    parameter_utils
    point_cloud_filter
    pose_graph_msgs
    visualization_msgs
    message_generation
    std_msgs
    pcl_ros)

  add_library(rosComponents INTERFACE IMPORTED)
  set_target_properties(rosComponents PROPERTIES
  INTERFACE_INCLUDE_DIRECTORIES "${catkin_INCLUDE_DIRS}"
  INTERFACE_LINK_LIBRARIS "${catkin_LIBRARIES}")
endif()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME})

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${ZLIB_INCLUDE_DIR}
)

add_definitions(${PCL_DEFINITIONS})

link_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
  ${ZLIB_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/LaserLoopClosure.cc)

target_link_libraries(${PROJECT_NAME}
  ${PCL_LIBRARIES}
  gtsam
  minizip
  ${Boost_LIBRARIES}
  ${ZLIB_LIBRARIES}
  ${catkin_LIBRARIES}
  rosComponents
  SESync 
)
