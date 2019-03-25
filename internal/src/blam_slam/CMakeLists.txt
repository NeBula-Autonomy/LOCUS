cmake_minimum_required(VERSION 3.1)
project(blam_slam)

# Need to do this to prevent Eigen conflict with SESync
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rosbag
  message_generation
  parameter_utils
  measurement_synchronizer
  point_cloud_filter
  point_cloud_odometry
  point_cloud_localization
  point_cloud_mapper
  geometry_utils
  pcl_ros
  pcl_conversions
  laser_loop_closure)

# find_package(SESync REQUIRED) # not elegant change if there's a way 

set(catkin_LIBRARIES ${catkin_LIBRARIES} gtsam)

add_service_files(FILES AddFactor.srv RemoveFactor.srv SaveGraph.srv)
generate_messages()

include(CheckCXXCompilerFlag)
check_cxx_compiler_flag("-std=c++14" COMPILER_SUPPORTS_CXX11)
check_cxx_compiler_flag("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if (COMPILER_SUPPORTS_CXX11)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
elseif(COMPILER_SUPPORTS_CXX0X)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
  message(FATAL_ERROR "${CMAKE_CXX_COMPILER} doesn't provide c++14 support.")
endif()

catkin_package(
INCLUDE_DIRS include
LIBRARIES ${PROJECT_NAME})

include_directories(include ${catkin_INCLUDE_DIRS})
link_directories(${catkin_LIBRARY_DIRS})
add_library(${PROJECT_NAME} src/BlamSlam.cc)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}.cc)
target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

add_executable(${PROJECT_NAME}_offline src/${PROJECT_NAME}_offline.cc)
target_link_libraries(${PROJECT_NAME}_offline
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)
