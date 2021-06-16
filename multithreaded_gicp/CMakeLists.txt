cmake_minimum_required(VERSION 2.8.3)
project(multithreaded_gicp)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  pcl_ros
  roscpp
  sensor_msgs
  frontend_utils
)

find_package(OpenMP)
# if (OpenMP_FOUND)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS} -fopenmp")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS} -fopenmp")
# endif()

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(test_same_output_different_num_threads test/test_same_output_different_num_threads.cpp)
target_link_libraries(test_same_output_different_num_threads
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
