cmake_minimum_required(VERSION 2.8.3)
project(measurement_synchronizer)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_ros
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS
    roscpp
    sensor_msgs
    pcl_ros
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

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_definitions(${catkin_DEFINITIONS})
add_library(${PROJECT_NAME} src/MeasurementSynchronizer.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)
