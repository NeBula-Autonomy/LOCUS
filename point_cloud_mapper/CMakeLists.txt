cmake_minimum_required(VERSION 2.8.3)
project(point_cloud_mapper)

#IF(NOT CMAKE_BUILD_TYPE)
#  SET(CMAKE_BUILD_TYPE Release)
#ENDIF()

SET(CMAKE_BUILD_TYPE Release)

find_package(PCL 1.7 REQUIRED COMPONENTS common octree filters)
if(NOT PCL_FOUND)
  message(FATAL_ERROR "This program requires the PCL library.")
endif(NOT PCL_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()


find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  core_msgs
  parameter_utils
  geometry_utils
  utils
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    std_msgs
    parameter_utils
    geometry_utils
    utils
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
)

link_directories(
  ${catkin_LIBRARY_DIRS}
  ${PCL_LIBRARY_DIRS}
)

add_library(${PROJECT_NAME} src/PointCloudMapper.cc
                            src/PointCloudMultiThreadedMapper.cc
                            src/SimplePointCloudMapper.cc
                            src/PointCloudIkdTreeMapper.cc
                            src/ikd_tree/ikd_Tree.cpp
                            )
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
  OpenMP::OpenMP_CXX
)

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

# Add Gtest for the package
if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} ${catkin_LIBRARIES})  
endif(CATKIN_ENABLE_TESTING)
