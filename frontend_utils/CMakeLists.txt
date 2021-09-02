cmake_minimum_required(VERSION 3.0.2)
project(frontend_utils)

add_compile_options(-std=c++11)
SET(CMAKE_BUILD_TYPE RelWithDebInfo)

find_package(catkin REQUIRED COMPONENTS
  roscpp pcl_ros
)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES frontend_utils
#  CATKIN_DEPENDS roscpp pcl_ros
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

#############
## Testing ##
#############
if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_gtest(utest test/test_common_functions.cpp)
  #add_rostest_gtest(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test test/test_common_functions.cpp)
  target_link_libraries(utest  ${catkin_LIBRARIES} -fopenmp)
endif()
